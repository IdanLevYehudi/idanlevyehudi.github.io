---
# --- Front-matter ---
title:  "From EKF-SLAM to iSAM2 - Part 3 (GraphSLAM)"
date:   2026-02-27 11:00 +0300
excerpt: "Going through major advancements in SLAM history - part three focuses on GraphSLAM"

# Navigation helpers
# categories: [robotics]              # broad buckets (lowercase, no spaces)
# tags:       [ROS, tutorial]         # fine-grained labels

# Page options (override the site defaults if needed)
layout: single
author_profile: true
read_time: true
comments: true
share: true

header:
  # teaser: /assets/images/2025-06-22-my-first-post/teaser.jpg   # ~400x250
  # overlay_image: /assets/images/2025-06-22-my-first-post/hero.jpg
  overlay_filter: 0.2     # darken for text readability (0-1)

# Table of contents
toc: true
toc_label: "On this page"
toc_icon: "list"

last_modified_at: 2026-02-27 11:00 +0300
---

<!-- Keep notations consistent - x for robot poses, m for landmarks, z for measurements, u for controls. Use subscripts for time indices, superscripts for landmark indices. -->

# 1 &middot; SLAM with Visual Landmarks

In the [previous post](https://idanlevyehudi.github.io/blog/ekf-slam-to-isam2-part-2/), we ended with the seeming success of **FastSLAM** and particle-based mapping. SLAM seemed like a solved problem, and researchers were expecting only incremental improvements in the upcoming years.

Then something big happened.

Around the mid-2000s, the community started to look at SLAM less like **"a filter that runs forever"** and more like **"a giant optimization problem"**: store *all* measurements, build a graph of constraints, and solve for the entire robot trajectory and map in one shot.

Remember what I said about EKF-SLAM in the [first post](https://idanlevyehudi.github.io/blog/ekf-slam-to-isam2-part-1/)?
> Viewed through today's "data-centric" lens, this feels natural: more data leads to better estimates.
> But in the 1980s the idea of coupling the two problems tightly was radical; researchers were used to solving them in separate phases.

This same principle applies here!

That viewpoint is what people now call **GraphSLAM** (and we'll also cover how it relates to *factor graphs*, *pose graphs*, and *smoothing*).
It became the backbone of most modern SLAM systems because it scales well and plays nicely with sparse linear algebra.

In this post we'll do two things:

1. Turn the SLAM posterior into a **non-linear least squares** problem (the "non-linear program" part).
2. Show how GraphSLAM solves it by repeatedly reducing it to a **linear least squares** system (the "linear program" part), and solving that efficiently with sparsity tricks.

For those who want a deeper technical dive, I always recommend "going back to the sources" and reading the original paper by Thrun et al. (2006)[^Thrun06].

[^Thrun06]: Sebastian Thrun and Michael Montemerlo. *The GraphSLAM Algorithm with Applications to Large-Scale Mapping of Urban Structures*. International Journal of Robotics Research, 25(5-6), 2006.

---

# 2 &middot; GraphSLAM - Back to the Problem Definition

We still start from the same SLAM posterior as in Parts 1-2.

For trajectory \$x \_ {0\:k}\$, landmarks \$m^{1\:L}\$, measurements \$z \_ {1\:k}\$, and controls \$u \_ {0\:k-1}\$, the posterior is

$$
p(x_{0:k}, m^{1:L} \mid z_{1:k}, u_{0:k-1}).
$$

Before we continue, let's keep four definitions explicit and consistent:

1. **Filtering** solves only for the latest robot state:
   $$
   p(x_t \mid z_{1:t}, u_{0:t-1}).
   $$
2. **Smoothing** solves for the full robot trajectory:
   $$
   p(x_{0:t} \mid z_{1:t}, u_{0:t-1}).
   $$
3. **Landmarks are static** in this post, so they do not evolve with time index; when landmarks are part of the state, we solve for the whole landmark set \$m^{1:L}\$ jointly.
4. **Full-SLAM vs Pose-SLAM**:
   - **Full-SLAM** estimates poses + landmarks (for example \$x_{0:t}, m^{1:L}\$).
   - **Pose-SLAM** optimizes only poses in the graph; landmarks are marginalized/eliminated or reconstructed afterward as a mapping by-product.

GraphSLAM is usually presented as a **smoothing** view, and in this post we start from the **full-SLAM smoothing** objective (poses + landmarks), then later eliminate landmarks computationally to solve a reduced pose system efficiently.

Why would we ever want to estimate the *entire past*? Because **loop closures** and late information matter. If at time \$k\$ the robot recognizes a place from time \$k-10\$, that new constraint should "pull" the whole trajectory into consistency. Smoothing makes this kind of global correction natural. The key insight of GraphSLAM was that keeping the whole trajectory in memory allows for *efficient solution of the whole history using sparse linear algebra methods*, overcoming the limitations of EKF-SLAM and FastSLAM.

## 2.1 &middot; "One Best Answer" Instead of the Whole Distribution

In Parts 1-2 we mostly focused on representing (exactly or approximately) a **full posterior distribution**:
- EKF-SLAM tracked a joint Gaussian over state variables.
- FastSLAM tracked a weighted particle approximation of the posterior.

That distribution contains much more than a single answer: it encodes uncertainty, multimodality, and correlations.  
But for many downstream tasks we often consume just one configuration (a trajectory + map estimate), not the whole density object.

<figure style="text-align:center;">
  <img
    src="/assets/images/posts/2026-02-27-ekf-slam-to-isam2-part-3/distribution_vs_point_estimate.svg"
    alt="Full distribution view versus point estimate view"
    style="display:block; margin:0 auto; max-width: 900px; width:100%; height:auto;"
  >
  <figcaption
    style="
      display: table;
      margin: 0 auto;
      font-size: 0.9em;
      text-align: center;
    "
  >
    Full posterior view keeps multiple plausible states, while MAP keeps one representative solution.
  </figcaption>
</figure>

Common point estimates:

1. **MMSE / posterior mean:**  
   \$x_{\text{MMSE}} = \mathbb{E}[x]\$ (minimizes expected squared error).  
   Intuition: this is the "center of mass" of the posterior; it is risk-optimal under quadratic loss, but can lie in a low-probability region if the posterior is multi-modal.
2. **MAP (maximum a-posteriori):**  
   \$x_{\text{MAP}} = \operatorname{argmax}_x \; p(x \mid \text{data})\$.  
   Intuition: this picks the single most probable configuration (the mode), which is exactly the form that leads naturally to GraphSLAM's optimization objective.

For GraphSLAM we'll pursue the **MAP** solution:

$$
\left(x_{0:k}, m^{1:L}\right)^*
=
\operatorname{argmax}_{x_{0:k},\,m^{1:L}}
p(x_{0:k}, m^{1:L} \mid z_{1:k}, u_{0:k-1}).
$$

We use $(\cdot)^*$ to denote an optimizer / point estimate (for example, the MAP solution).

> **Intuition:** MAP finds the trajectory+map that makes *all* measurements "as unsurprising as possible".

---

# 3 &middot; SLAM Factorization for MAP

The posterior itself is intractable, but for MAP we can rewrite it into something we *can* optimize.

Using Bayes' rule,

$$
p(x_{0:k}, m^{1:L} \mid z_{1:k}, u_{0:k-1})
=
\frac{
p(z_{1:k} \mid x_{0:k}, m^{1:L}, u_{0:k-1}) \; p(x_{0:k}, m^{1:L} \mid u_{0:k-1})
}{
p(z_{1:k} \mid u_{0:k-1})
}.
$$

I know this may look daunting, but it is just a simple application of \$p(A \mid B) = \frac{p(B \mid A) p(A)}{p(B)}\$.

The denominator \$p(z_{1:k} \mid u_{0:k-1})\$ does **not** depend on \$x_{0:k}\$ or \$m^{1:L}\$, so for **argmax** we can ignore it (it's just a constant scale).

Now apply the standard SLAM conditional independences:

- **Motion model:** state depends only on previous state + control, i.e: \$p(x_{t+1} \mid x_{0:t}, u_{0:t}) = p(x_{t+1} \mid x_t, u_t)\$.
- **Measurement model:** measurement depends only on same-time state, meaning each \$z_t^i\$ depends only on \$x_t\$ and the observed landmark.

With known correspondences (data association), let \$c_t^i\$ be the landmark index that generated measurement \$z_t^i\$. Let \$M_t\$ be the set of all measurements at time \$t\$. The MAP objective becomes:

$$
\operatorname{argmax}_{x_{0:k},\, m^{1:L}}
\Bigg[
p(x_0)\;
\prod_{t=0}^{k-1} p(x_{t+1}\mid x_t, u_t)\;
\prod_{t=1}^{k} \prod_{i\in M_t} p(z_t^i \mid x_t, m^{c_t^i})
\Bigg].
$$

Notice how we've transformed the unknown MAP objective of the Full-SLAM to a product of known models!

## 3.1 &middot; Connection to a Graph

A **factor graph** is a bipartite graph with two node types: variable nodes (unknowns we solve for) and factor nodes (local functions over a small subset of variables).
The global objective to solve is the product of all factors, which is exactly the structure we got from the Full-SLAM MAP factorization above.

Let's look at this "hello-world" factor graph:

<figure style="text-align:center;">
  <img
    src="/assets/images/posts/2026-02-27-ekf-slam-to-isam2-part-3/factor_graph_hello_world.svg"
    alt="Minimal factor graph with variables x1, x2 and factors f1, f12"
    style="display:block; margin:0 auto; max-width: 760px; width:100%; height:auto;"
  >
</figure>

The joint density is $p(x1, x2) \propto f1(x1)\cdot f12(x1, x2)$, up to normalization.

The factorization of the Full-SLAM MAP objective is a factor graph hiding in plain sight:

- Variable nodes: robot poses \$x_0,\dots,x_k\$ and landmarks \$m^1,\dots,m^L\$.
- Factor nodes:
  - motion factor \$f_t^{\text{motion}}(x_t,x_{t+1})\$,
  - measurement factor \$f_{t,i}^{\text{meas}}(x_t,m^{c_t^i})\$,
  - prior factor \$f_0^{\text{prior}}(x_0)\$.

A tiny example (two poses, one landmark), with explicit factor nodes:

<figure style="text-align:center;">
  <img
    src="/assets/images/posts/2026-02-27-ekf-slam-to-isam2-part-3/factor_graph_tiny_full_slam.svg"
    alt="Tiny Full-SLAM factor graph with x0, x1 and m1"
    style="display:block; margin:0 auto; max-width: 760px; width:100%; height:auto;"
  >
  <figcaption
    style="
      display: table;
      margin: 0 auto;
      font-size: 0.9em;
      text-align: center;
    "
  >
    Tiny Full-SLAM graph with one prior factor, one motion factor, and two measurement factors.
  </figcaption>
</figure>

GraphSLAM's core idea is to build this graph whenever new robot poses and measurements are registered, and solve it.

But how exactly do we solve a factor graph's MAP?

---

# 4 &middot; From MAP to Non-Linear Least Squares

We'll do this in two steps: first a general log transform that does not assume any specific noise model, then a Gaussian specialization that turns each term into a squared residual.

## 4.1 &middot; Taking Logs Turns Products into Sums

Because log is monotone, maximizing the posterior is equivalent to maximizing the log posterior:

$$
\operatorname{argmax}\; p(\cdot)
\;=\;
\operatorname{argmax}\; \log p(\cdot).
$$

And since
\$\log \prod_i a_i = \sum_i \log a_i\$, the huge product becomes a sum.

Finally, maximizing log-probability is equivalent to minimizing **negative** log-probability:

$$
\operatorname{argmax}_x\; f(x)
\;=\;
\operatorname{argmin}_x\; \big(-\log f(x)\big).
$$

## 4.2 &middot; Additive Gaussian Noise Models

Now we assume (as in EKF-SLAM) additive Gaussian noise:

1. **Motion model**
   $$
   x_{t+1} = g(x_t, u_t) + w_t,\qquad w_t \sim \mathcal{N}(0, R_t).
   $$
2. **Measurement model**
   $$
   z_t^i = h(x_t, m^{c_t^i}) + v_t^i,\qquad v_t^i \sim \mathcal{N}(0, Q_t).
   $$

Where \$g(\cdot)\$ and \$h(\cdot)\$ can be *non-linear*.

We'll write each factor in terms of a residual (prediction error). In general:
\$r(\theta) = y - \hat y(\theta)\$, where \$\theta\$ are the unknown variables we optimize.
In SLAM, examples are \$x_{t+1} - g(x_t,u_t)\$ and \$z_t^i - h(x_t,m^{c_t^i})\$.

We'll also use the covariance-form Mahalanobis norm:

$$
\left\Vert e \right\Vert_{\Sigma}^2 \;\triangleq\; e^\top \Sigma^{-1} e,
$$

With our convenient additive Gaussian noise, each factor contributes (up to an additive constant)
$$
-\log p(y \mid \theta) = \frac12 \left\Vert r(\theta) \right\Vert_{\Sigma}^2 + \text{const}.
$$.

<details markdown="1">
<summary>Derivation:</summary>

If \$x = \mu + w\$ with \$w \sim \mathcal{N}(0,\Sigma)\$, then \$x \sim \mathcal{N}(\mu,\Sigma)\$ and

$$
p(x) =
(2\pi)^{-\frac{d}{2}} |\Sigma|^{-\frac12}
\exp\left(
-\frac12 (x-\mu)^\top \Sigma^{-1}(x-\mu)
\right),
$$

where \$d\$ is the dimension of \$x\$.

So if \$x_{t+1} = g(x_t,u_t) + w_t\$, then

$$
p(x_{t+1}\mid x_t,u_t)
\propto
\exp\left(
-\frac12 \left\Vert x_{t+1} - g(x_t,u_t)\right\Vert_{R_t}^2
\right).
$$

Same story for measurements:
\$p(z_t^i \mid x_t, m^{c_t^i}) \propto \exp\left(-\frac12 \left\Vert z_t^i - h(x_t,m^{c_t^i})\right\Vert_{Q_t}^2\right)\$.

</details>

## 4.3 &middot; The GraphSLAM Cost Function

Dropping constants that don't change the argmin, GraphSLAM ends up minimizing a sum of squared residuals:

$$
\boxed{
\begin{aligned}
J_{\text{GraphSLAM}}(x_{0:k}, m^{1:L})
&=
\left\Vert x_0 - \bar x_0 \right\Vert_{\Sigma_0}^2
\\
&\quad + \sum_{t=0}^{k-1}
\left\Vert x_{t+1} - g(x_t, u_t)\right\Vert_{R_t}^2
\\
&\quad + \sum_{t=1}^{k} \sum_{i\in M_t}
\left\Vert z_t^i - h(x_t, m^{c_t^i})\right\Vert_{Q_t}^2.
\end{aligned}}
$$

This is the **non-linear least squares** problem GraphSLAM solves.

> **Spring model intuition from the paper[^Thrun06]:**  
> Each squared term is like a spring:
> - Motion springs connect consecutive poses.
> - Measurement springs connect poses to landmarks.
> - The prior spring anchors the whole thing so it doesn't "float" in space.
>
> Solving the optimization is like finding robot and landmarks positions resulting in an equilibrium of spring tensions.

---

# 5 &middot; Solving Non-Linear Least Squares (by Linearization)

GraphSLAM's cost \$J_{\text{GraphSLAM}}(x_{0:k}, m^{1:L})\$ is a **non-linear** least squares problem: it is a sum of squared residuals, but the residuals depend on the unknowns through non-linear functions (\$g,h\$).

To avoid clashing with Jacobians later, we reserve \$J_{\text{GraphSLAM}}\$ for this scalar cost, and use \$J_j\$ for Jacobian matrices.

The standard way to solve problems of this form is **Gauss-Newton**, which iteratively performs:

1. Linearize the residuals around the current guess.
2. Solve the resulting **linear** least squares for an increment \$\Delta\$.
3. Update the guess.

There are many algorithmic variants of the basic idea of "linearize + iterate", and they will be important when we'll cover more sophisticated SLAM algorithms in the upcoming posts.

## 5.1 &middot; Gauss-Newton: Linearize Around the Current Estimate

We can write a generic non-linear least squares objective as:

$$
\min_y \;
\sum_{j} \left\Vert r_j(y)\right\Vert_{\Sigma_j}^2,
$$

where \$y\$ stacks **all** unknown variables and each \$r_j\$ is one residual (in SLAM: prior, motion, or measurement).

At the current guess \$\mu\$, define an increment \$\Delta \triangleq y-\mu\$ and linearize each residual:

$$
r_j(\mu+\Delta) \approx r_j(\mu) + J_j \Delta,
\qquad
J_j = \left.\frac{\partial r_j}{\partial y}\right|_{y=\mu}.
$$

Plugging this into the objective produces a **linear** least squares problem in \$\Delta\$:

$$
\min_\Delta \sum_j \left\Vert J_j \Delta + r_j(\mu) \right\Vert_{\Sigma_j}^2.
$$

## 5.2 &middot; From Linear Least Squares to Back-Substitution

In Section 5.1 we wrote the linearization as $y=\mu+\Delta$, where $\Delta$ is the increment from the current guess $\mu$ for the stacked variable vector $y$.
In this linearized step, we solve for the optimal increment $\Delta^\*$ and then update $\mu \leftarrow \mu + \Delta^\*$.

The optimum $\Delta^*$ satisfies the normal equations, which we can write as a sparse linear system:

$$
\boxed{
\Omega \Delta = \xi,
\qquad
\Omega \triangleq \sum_j J_j^\top \Sigma_j^{-1} J_j,
\qquad
\xi \triangleq -\sum_j J_j^\top \Sigma_j^{-1} r_j(\mu).
}
$$

Here $\Omega$ is the **information matrix** (a Gauss-Newton approximation to the Hessian), and $\xi$ is the **information vector** (the accumulated, weighted residual term). Concretely, $\xi$ is built from the Jacobians and residuals $r_j(\mu)$, and it is the right-hand side of the normal equations.

Equivalently, the linearized objective is the quadratic form:

$$
\frac12 \Delta^\top \Omega \Delta - \Delta^\top \xi + \text{const}.
$$

This quadratic can be viewed as the negative log of an (unnormalized) Gaussian density over $\Delta$. In **information form**:

$$
p(\Delta) \propto
\exp\left(
-\frac12 \Delta^\top \Omega \Delta + \Delta^\top \xi
\right),
$$

If this looks unfamiliar, compare it to the usual covariance form:

$$
p(\Delta)=\mathcal{N}(\Delta;\Delta^*,\Sigma)
\propto
\exp\left(
-\frac12(\Delta-\Delta^*)^\top \Sigma^{-1}(\Delta-\Delta^*)
\right).
$$

Expanding the exponent and matching terms shows:

$$
\Omega=\Sigma^{-1},
\qquad
\xi=\Sigma^{-1}\Delta^*,
$$

so the mean (and MAP estimate) \$ \Delta^* \$ satisfies $\Omega\Delta^*=\xi$ (and $\Sigma=\Omega^{-1}$).

Crucially, we do **not** compute $\Omega^{-1}$. We solve the sparse linear system using a factorization.

### Back-Substitution: How \$\Delta\$ Is Recovered

To recover $\Delta^*$ we need to solve the sparse linear system $\Omega\Delta=\xi$. Inverting $\Omega$ explicitly would be slow and numerically fragile, so we avoid it and solve via a factorization instead.

In SLAM, once we anchor the reference frame (for example, by adding a prior on the first pose) and the problem is sufficiently constrained, $\Omega$ is (block) symmetric positive definite. In that case, a common solution method is **Cholesky factorization**, which factorizes $\Omega$ into a triangular factor and its transpose.

Why do we bother with the triangular factor? Because triangular systems let us solve one new unknown at a time.

$$
\Omega = R^\top R,
$$

where \$R\$ is upper-triangular (equivalently, one can write \$\Omega=LL^\top\$ with \$L\$ lower-triangular). Once we have \$R\$, solving \$\Omega\Delta=\xi\$ reduces to two triangular solves.

Then compute the increment via two triangular solves:

$$
R^\top u = \xi \quad\text{(forward substitution)},\qquad
R\,\Delta = u \quad\text{(back substitution)}.
$$

<details markdown="1">
<summary>3x3 example</summary>

Suppose

$$
\Omega =
\begin{bmatrix}
4 & 2 & 0\\
2 & 5 & 2\\
0 & 2 & 2
\end{bmatrix},
\qquad
\xi =
\begin{bmatrix}
10\\13\\6
\end{bmatrix}.
$$

A Cholesky factor is
$$
R =
\begin{bmatrix}
2 & 1 & 0\\
0 & 2 & 1\\
0 & 0 & 1
\end{bmatrix},
\qquad
R^\top =
\begin{bmatrix}
2 & 0 & 0\\
1 & 2 & 0\\
0 & 1 & 1
\end{bmatrix},
\qquad
\Omega = R^\top R.
$$

The triangular system allows us to solve \$R^\top u=\xi\$ top-to-bottom (forward substitution), then solve \$R\Delta=u\$ bottom-to-top (back substitution).

1. Solve \$R^\top u=\xi\$:

   $$
   \begin{bmatrix}
   2 & 0 & 0\\
   1 & 2 & 0\\
   0 & 1 & 1
   \end{bmatrix}
   \begin{bmatrix}
   u_1\\u_2\\u_3
   \end{bmatrix}
   =
   \begin{bmatrix}
   10\\13\\6
   \end{bmatrix}
   $$

   $$
   \begin{aligned}
   2u_1 &= 10 &\Rightarrow\; u_1 &= 5\\
   u_1 + 2u_2 &= 13 &\Rightarrow\; u_2 &= 4\\
   u_2 + u_3 &= 6 &\Rightarrow\; u_3 &= 2
   \end{aligned}
   $$

2. Solve \$R\Delta=u\$:

   $$
   \begin{bmatrix}
   2 & 1 & 0\\
   0 & 2 & 1\\
   0 & 0 & 1
   \end{bmatrix}
   \begin{bmatrix}
   \Delta_1\\\Delta_2\\\Delta_3
   \end{bmatrix}
   =
   \begin{bmatrix}
   5\\4\\2
   \end{bmatrix}
   $$

   $$
   \begin{aligned}
   \Delta_3 &= 2\\
   2\Delta_2 + \Delta_3 &= 4 &\Rightarrow\; \Delta_2 &= 1\\
   2\Delta_1 + \Delta_2 &= 5 &\Rightarrow\; \Delta_1 &= 2
   \end{aligned}
   $$

So \$\Delta=[2,1,2]^\top\$.

</details>

Dense Cholesky costs \$O(n^3)\$ for an \$n\times n\$ matrix. In SLAM, the important point is that \$\Omega\$ is sparse, and sparse Cholesky can be much faster (the cost depends on fill-in and variable ordering).

Finally update \$\mu \leftarrow \mu + \Delta\$ and repeat until convergence.

In the next section we'll apply this to GraphSLAM, and see how sparsity and landmark elimination make the linear solves tractable.

# 6 &middot; How GraphSLAM Actually Solves It

Section 5 described the "generic" Gauss-Newton recipe.
GraphSLAM is that recipe specialized to SLAM, plus one key computational trick: build the linearized problem in sparse information form ($\Omega,\xi$), then eliminate landmarks so the expensive solve is pose-only.

GraphSLAM is a small set of routines that run in a loop:

- `GraphSLAM_initialize`: build an initial guess for the robot trajectory.
- `GraphSLAM_linearize`: linearize all constraints around the current guess, and build a sparse information matrix $\Omega$ and information vector $\xi$ for the full problem (poses + landmarks).
- `GraphSLAM_reduce`: eliminate (marginalize) all landmarks, producing a smaller pose-only system $(\tilde\Omega, \tilde\xi)$.
- `GraphSLAM_solve`: solve the pose-only system for the pose mean (and covariance), then recover the landmark means.
- `GraphSLAM_known_correspondence`: the outer loop that repeats linearize -> reduce -> solve until convergence.

In each iteration we "linearize -> reduce -> solve -> update" the guess until convergence. The original paper reports convergence often after 2-3 iterations[^Thrun06].

## 6.1 &middot; `GraphSLAM_reduce`: Marginalize Landmarks (Variable Elimination)

After linearization, the full problem (poses + landmarks) is a joint Gaussian in information form:

$$
p(x,m) \propto \exp\left(-\frac12
\begin{bmatrix}x\\m\end{bmatrix}^\top
\Omega
\begin{bmatrix}x\\m\end{bmatrix}
\;+\;
\begin{bmatrix}x\\m\end{bmatrix}^\top
\begin{bmatrix}\xi_x\\\xi_m\end{bmatrix}
\right).
$$

Solving the full system over both poses and landmarks can still be expensive when there are many landmarks. GraphSLAM's key trick is to eliminate landmarks analytically and solve a smaller pose-only system first (without throwing away any information about the trajectory).

GraphSLAM makes this tractable by applying two standard Gaussian operations (both analytic after linearization) [^Thrun06]:

1. **Marginalize / eliminate landmarks** to obtain a pose-only Gaussian over $x$ (this is `GraphSLAM_reduce`).
2. **Condition on the pose estimate** to recover landmark means (this is the back-substitution part of `GraphSLAM_solve`).

To write these operations in the same notation as our linear system, we partition the information matrix and information vector into pose and landmark blocks:

$$
\Omega =
\begin{bmatrix}
\Omega_{xx} & \Omega_{xm}\\
\Omega_{mx} & \Omega_{mm}
\end{bmatrix},
\qquad
\xi=
\begin{bmatrix}
\xi_x\\
\xi_m
\end{bmatrix}.
$$

The mean (and MAP estimate) satisfies the linear system $\Omega\begin{bmatrix}x \\\ m\end{bmatrix}=\xi$, i.e.

$$
\Omega_{xx}x + \Omega_{xm}m = \xi_x,
\qquad
\Omega_{mx}x + \Omega_{mm}m = \xi_m.
$$

A convenient way to derive both steps is to eliminate $m$ from the normal equations. Concretely, suppose we temporarily treat $x$ as known. Then the conditional over landmarks is

$$
p(m \mid x) \propto \exp\left(-\frac12\,m^\top\Omega_{mm}m + m^\top(\xi_m-\Omega_{mx}x)\right).
$$

This is a Gaussian with information matrix $\Omega_{mm}$ and information vector $\xi_m-\Omega_{mx}x$, so its mean (also the conditional MAP) is the "best-fit landmarks given poses":

$$
m^*(x) = \Omega_{mm}^{-1}(\xi_m-\Omega_{mx}x),
$$

We will use this expression in two ways:
1. Substitute it into the pose equation to obtain a reduced system over $x$ alone.
2. After solving that reduced system for $x^\*$, plug $x=x^\*$ into $m^\*(x)$ to recover landmark means (back-substitution).

Now we plug $m^\*(x)$ back into the pose equation. This yields the reduced pose-only system:

$$
\boxed{
\tilde\Omega\,x=\tilde\xi,
\qquad
\tilde\Omega=\Omega_{xx}-\Omega_{xm}\Omega_{mm}^{-1}\Omega_{mx},
\qquad
\tilde\xi=\xi_x-\Omega_{xm}\Omega_{mm}^{-1}\xi_m.
}
$$

The reduced pair $(\tilde\Omega,\tilde\xi)$ is the information-form marginal over poses $p(x)$. Algebraically, it is the Schur complement of $\Omega_{mm}$ in $\Omega$. [^Thrun06]

<details markdown="1">
<summary>The same operations can be done in covariance form:</summary>

If we instead write the joint as

$$
\begin{bmatrix}x\\m\end{bmatrix}
\sim
\mathcal{N}\left(
\begin{bmatrix}\mu_x\\\mu_m\end{bmatrix},
\begin{bmatrix}
\Sigma_{xx} & \Sigma_{xm}\\
\Sigma_{mx} & \Sigma_{mm}
\end{bmatrix}
\right),
$$

then marginalization is just "take the block":

$$
p(x)=\mathcal{N}(\mu_x,\Sigma_{xx}).
$$

Conditioning is also analytic:

$$
p(m \mid x)=
\mathcal{N}\left(
\mu_m+\Sigma_{mx}\Sigma_{xx}^{-1}(x-\mu_x),
\;\Sigma_{mm}-\Sigma_{mx}\Sigma_{xx}^{-1}\Sigma_{xm}
\right).
$$

</details>

> **Remark (EKF connection).** The EKF uses the same two Gaussian operations. In the predict step, we combine the previous posterior with the motion model to form a joint over $(x_t, x_{t+1})$, then marginalize $x_t$ to get the prior over $x_{t+1}$. In the update step, we condition on the new measurement; see the [first post](https://idanlevyehudi.github.io/blog/ekf-slam-to-isam2-part-1/).

### Why Reduction Is Efficient in SLAM

The elimination formulas above tell us exactly what work `GraphSLAM_reduce` has to do: for each landmark, we invert its landmark block and apply a Schur-complement update to the pose blocks it touches.

In a generic Gaussian problem, $\Omega_{mm}$ could be large and dense, and then this "reduction" would be as expensive as solving the full system. In SLAM, the factor-graph structure makes it cheap:

- Each measurement factor touches one pose and one landmark, so there are no landmark-landmark couplings.
- Therefore $\Omega_{mm}$ is block-diagonal: one small block per landmark. [^Thrun06]
- Eliminating one landmark only affects the poses that observed it; it creates new pose-pose couplings among those poses (fill-in), but it does not touch unrelated parts of the trajectory.

<details markdown="1">
<summary>Worked example: eliminate one landmark and see the fill-in</summary>

Let's look at a tiny (scalar) example with two pose variables \$x=[x_1,x_2]^\top\$ and one landmark variable \$m\$.
Suppose the linearized normal equations are

$$
\Omega
\begin{bmatrix}x\\m\end{bmatrix}
=
\xi,
\qquad
\Omega =
\begin{bmatrix}
4 & 0 & 2\\
0 & 3 & 1\\
2 & 1 & 5
\end{bmatrix},
\qquad
\xi =
\begin{bmatrix}
10\\
9\\
19
\end{bmatrix}.
$$

Here the pose-pose block is diagonal (\$x_1\$ and \$x_2\$ are not directly coupled), but both poses connect to the landmark through the last column/row. Partition \$(\Omega,\xi)\$ as poses-first:

$$
\Omega_{xx}=
\begin{bmatrix}4&0\\0&3\end{bmatrix},
\qquad
\Omega_{xm}=
\begin{bmatrix}2\\1\end{bmatrix},
\qquad
\Omega_{mm}=[5],
\qquad
\xi_x=
\begin{bmatrix}10\\9\end{bmatrix},
\quad
\xi_m=[19].
$$

Eliminating \$m\$ gives the reduced pose-only system:

$$
\tilde\Omega
=
\Omega_{xx}-\Omega_{xm}\Omega_{mm}^{-1}\Omega_{mx}
=
\begin{bmatrix}
16/5 & -2/5\\
-2/5 & 14/5
\end{bmatrix},
\qquad
\tilde\xi
=
\xi_x-\Omega_{xm}\Omega_{mm}^{-1}\xi_m
=
\begin{bmatrix}
12/5\\
26/5
\end{bmatrix}.
$$

Notice the key effect: eliminating the landmark introduced an off-diagonal coupling (\$-2/5\$) between \$x_1\$ and \$x_2\$.
This is the "fill-in" created by elimination.

Solving \$\tilde\Omega\,x=\tilde\xi\$ yields \$x=[1,2]^\top\$.
Then recover the eliminated variable by back-substitution:

$$
m = \Omega_{mm}^{-1}\left(\xi_m - \Omega_{mx}x\right)
=
\frac{1}{5}\left(19 - [2\;\;1]\begin{bmatrix}1\\2\end{bmatrix}\right)
= 3.
$$

In real GraphSLAM, each landmark block \$\Omega_{j,j}\$ is a tiny \$2\times2\$ (for 2D point landmarks), so this same elimination step stays cheap per landmark.

</details>

Let $\tau(j)$ be the set of poses that observed landmark $m^j$:

$$
\tau(j) = \{\,t \;|\; \exists i \text{ such that } c_t^i = j \,\}.
$$

Then eliminating landmark $m^j$ updates only the pose blocks in $\tau(j)$:

$$
\boxed{
\begin{aligned}
\tilde\Omega_{\tau(j),\tau(j)}
&\;{-}{=}\;
\Omega_{\tau(j),j}\,\Omega_{j,j}^{-1}\,\Omega_{j,\tau(j)},
\\
\tilde\xi_{\tau(j)}
&\;{-}{=}\;
\Omega_{\tau(j),j}\,\Omega_{j,j}^{-1}\,\xi_j.
\end{aligned}}
$$

After this update, we can remove the rows and columns of $m^j$.

## 6.2 &middot; Solve Poses, Then Recover Landmark Means

After reduction, we solve the pose-only sparse linear system for the pose estimate $x^*$ (the mean / MAP of the reduced Gaussian):

$$
\tilde\Omega\,x^*=\tilde\xi,
$$

using sparse Cholesky (Section 5.2). For extremely large problems, the paper also notes that iterative solvers like conjugate gradient can be used when we only need matrix-vector products. [^Thrun06]

We can now recover landmarks by back-substitution; this is exactly conditioning the joint Gaussian on the solved poses. From the landmark block row

$$
\Omega_{mx}x^*+\Omega_{mm}m^*=\xi_m,
$$

we get:

$$
\boxed{
m^*=\Omega_{mm}^{-1}\left(\xi_m-\Omega_{mx}x^*\right).
}
$$

Because $\Omega_{mm}$ is block-diagonal, each landmark mean is an independent tiny solve. For one landmark $m^j$:

Equivalently, for a single landmark $m^j$ connected only to the poses in $\tau(j)$, the same back-substitution becomes:

$$
\boxed{
m^{j*}=
\Omega_{j,j}^{-1}\Big(
\xi_j-\Omega_{j,\tau(j)}\,x^*_{\tau(j)}
\Big).
}
$$

GraphSLAM does not compute the full landmark covariance (and landmark-landmark correlations), because that would be quadratic in the number of landmarks.

## 6.3 &middot; Unknown Correspondences

So far we assumed we know which landmark generated each measurement ($c_t^i$ known). In reality this is hard.
GraphSLAM proposes a greedy correspondence search on top of the solver:

- start with a conservative hypothesis where every observation is its own landmark (no merges),
- repeatedly test if two landmarks are likely to be the same physical feature,
- if so, merge them, then rerun GraphSLAM.

### The Correspondence Test

To decide if $m^j$ and $m^k$ should be merged, the paper computes the probability that their difference is zero. This test needs pose uncertainty (at least some covariance blocks), which is why GraphSLAM tracks / computes it for the pose estimate. [^Thrun06]

We build a Gaussian for the difference between two landmark estimates, and merge them if "difference = 0" is likely enough:

1. Compute the marginal Gaussian posterior over the pair $(m^j, m^k)$ (this requires the pose covariance).
2. Define the difference variable $\Delta_{j,k} = m^j - m^k$.
3. Evaluate $p(\Delta_{j,k} = 0)$, and compare to a threshold. If the probability is high enough, merge.

<details markdown="1">
<summary>Sketch of the math</summary>

Let $\tau(j,k) = \tau(j)\cup\tau(k)$ be the poses that observed either feature.

Given the full information form $(\Omega,\xi)$ and the pose covariance $\tilde\Sigma_{0:k}$, the paper derives an information form for the joint marginal over $(m^j, m^k)$:

$$
\Omega_{[j,k]}
=
\Omega_{jk,jk}
-
\Omega_{jk,\tau(j,k)}\,
\tilde\Sigma_{\tau(j,k),\tau(j,k)}\,
\Omega_{\tau(j,k),jk},
$$

and then sets $\xi_{[j,k]} = \Omega_{[j,k]}\,\mu_{[j,k]}$ (they compute $\mu_{[j,k]}$ from the map estimate).

Then define

$$
\Delta_{j,k} = m^j - m^k
=
\begin{bmatrix}1\\-1\end{bmatrix}^\top
\begin{bmatrix}m^j\\m^k\end{bmatrix}.
$$

This linear transform means $\Delta_{j,k}$ is also Gaussian. If we convert the pair marginal to moments form,
$$
\Sigma_{[j,k]}=\Omega_{[j,k]}^{-1},
\qquad
\mu_{[j,k]}=\Sigma_{[j,k]}\,\xi_{[j,k]},
$$
and define $A=\begin{bmatrix}I & -I\end{bmatrix}$, then
$$
\Delta_{j,k}\sim \mathcal{N}(\mu_\Delta,\Sigma_\Delta),
\qquad
\mu_\Delta=A\,\mu_{[j,k]},
\qquad
\Sigma_\Delta=A\,\Sigma_{[j,k]}A^\top.
$$

The "correspondence probability" in the paper is this Gaussian evaluated at zero, i.e. $p(\Delta_{j,k}=0)=\mathcal{N}(0;\mu_\Delta,\Sigma_\Delta)$. [^Thrun06]

</details>

### The Greedy Merge Loop

The paper's baseline greedy loop is:

- Start with every observation as its own landmark (no merges).
- Run GraphSLAM to get a map/trajectory estimate and the pose covariance needed for the test.
- Repeatedly test feature pairs $(j,k)$; if a merge passes the threshold, merge and rerun GraphSLAM.
- Stop when no merges are found. [^Thrun06]

The authors also note this is not an efficient implementation (it checks all pairs, and reruns the full solver after each merge), but it demonstrates that the graph-based formulation can support a data-association search loop cleanly.

---

# 7 &middot; From Offline GraphSLAM to Online Graphical SLAM

GraphSLAM was a big deal because it showed that **full SLAM** can be solved efficiently by:

- Exploiting **sparsity** (local constraints leading to sparse matrices).
- Using **variable elimination** (marginalizing landmarks) to avoid solving for everything at once.

The paper demonstrates very large-scale mapping (they discuss maps with up to \$10^8\$ features) and adding extra constraints like GPS naturally.
This was indeed a leap in the practicality of SLAM and closely-related Structure-from-Motion (SfM) algorithms, breaking a limit of the scale possible to map.

But GraphSLAM is still fundamentally an **offline** SLAM: each time a new robot pose or observation is made, we build a new batch problem and solve it from scratch.
If we want our robot to quickly process new measurements and use them to recalculate its trajectories in real-time, we'll pretty quickly hit a computational limit when mapping larger and larger areas.
What we need is a SLAM algorithm that operates **incrementally** and **online**.

That's exactly what the next algorithms in this series (iSAM / iSAM2) are designed to solve.

Stay tuned!
