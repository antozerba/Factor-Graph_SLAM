# TTK4250 Group Assignment 2 — Factor graph SLAM

Landmark SLAM on a factor graph, solved with GTSAM's iSAM2 back-end and a JCBB
front-end, on two datasets: a simulated one with ground truth, and the Victoria
Park lidar dataset.

## Setup

Install [Miniconda](https://www.anaconda.com/download) — the minimal installer,
**not** the full Anaconda Distribution, and not "whatever conda you already
have". Then, from this directory:

```sh
conda info                      # check: conda version 25 or newer
conda env create -f environment.yml
conda activate ttk4250_ga2
pip install -e . --no-deps
```

**That first line is not optional.** A conda older than 25 cannot solve this
environment, and the error it gives you is unrecognisable. If `conda version` is
older, or if `base environment` points at an Anaconda install you did not expect,
stop and read `docs/INSTALL.md`.

`pip install -e . --no-deps` puts `run_sim`, `run_real` and `plot_run` on your
path. Re-run it only if entry points change; editing source needs nothing.

Do not use `pip install gtsam`: PyPI has no Windows wheel for GTSAM.

## Running

From the root of this directory:

```sh
run_sim                      # simulated data
run_real                     # Victoria Park

run_sim --config configs/sim_default.yaml --steps 1000 --output-dir runs/sim/example
plot_run runs/sim/example    # re-plot a finished run
```

With the environment active. A run ends by opening its figures, and the
command returns once you close the windows. Add `--no-show-plots` to skip them
(for example over SSH); figures are written to the run directory either way.

Ctrl+C stops a run and still saves it up to the last completed step, marked as
aborted, so you can plot it with `plot_run`. Add `--no-save-on-abort` if you
would rather Ctrl+C just exit.

Every run writes its resolved configuration, per-step diagnostics and state
snapshots into its output directory, so a run can be re-plotted and compared
later without re-running it.

## What you implement

Ten functions, all marked `TODO` in the source and graded by `pytest`:

| | Function | File |
|---|---|---|
| a | `relative_pose` | `src/graphslam/preprocessing.py` |
| b | `preintegrate` | `src/graphslam/preprocessing.py` |
| c1 | `predict_pose` | `src/graphslam/factor_graph.py` |
| c2 | `add_odometry_factor` | `src/graphslam/factor_graph.py` |
| c3 | `add_landmark_factor` | `src/graphslam/factor_graph.py` |
| d | `predict_measurement` | `src/graphslam/factor_graph.py` |
| e | `assemble_joint_covariance` | `src/graphslam/factor_graph.py` |
| f | `innovation_covariance` | `src/graphslam/factor_graph.py` |
| g | `inverse_measurement` | `src/graphslam/factor_graph.py` |
| h | `TentativeLandmark.is_confirmed` | `src/graphslam/landmark_manager.py` |

Each part has its own tests, selected by a marker named after the part:

```sh
pytest -m b              # the tests for (b)
pytest -m c2 -x          # (c2), stop at the first failure
pytest                   # everything, including the end-to-end tests
```

Nothing else needs changing. `slam.py`, `data_association.py`, the loaders, the
logger and the plotting are given.

## Layout

```text
environment.yml   the conda environment; this is the supported setup
configs/          run configurations; everything marked "TODO tune" is yours
data/             simulated and Victoria Park datasets
docs/             INSTALL.md (setup) and GTSAM.md (the GTSAM calls you need)
src/graphslam/
  preprocessing.py     front-end: odometry and lidar          (a, b)
  factor_graph.py      factors, measurement model, covariance (c-g)
  landmark_manager.py  landmark birth, M-of-N                 (h)
  data_association.py  JCBB                                   given
  evaluation.py        NEES, NIS, map quality, run summary     given
  slam.py              the main loop                          given
  loaders/             dataset adapters                       given
  plotting/, plotter.py, logger.py                            given
tests/            the graded test suite
typings/          GTSAM type stubs, so your editor can show its signatures
create_handin.py  zips your code and configs for the hand-in
```

## Editor support

GTSAM is a compiled library, so on its own your editor cannot see what is
inside it. `typings/` holds type stubs generated from the installed GTSAM, which
VS Code (Pylance) picks up automatically when you open this folder as the
workspace: completion, argument names and types, and every overload. For
example, hovering `compose` shows that it optionally takes the two Jacobians
`H1` and `H2`. Select the `ttk4250_ga2` environment as the interpreter.

The stubs have signatures only, no descriptions. From Python, `help()` shows
the same signatures:

```sh
python -c "import gtsam; help(gtsam.Pose2.compose)"
```

**`docs/GTSAM.md` lists every GTSAM call Task 1 needs, with what each one
means.** Start there; the assignment text names the calls each part needs.
GTSAM's own documentation (https://gtsam.org) is written for C++ and rarely
needed.

## Reading a run

Every run prints a summary as soon as it is saved, before any figure opens.
`plot_run` prints it too, and it is stored under `"summary"` in the run's
`metadata.json`, so runs can be compared without plotting them:

```text
Map quality: 72 of 78 observed true landmarks mapped, 6 missed, 0 duplicates, 3 spurious, RMS position error 1.89 m
Summary: RMSE x 0.72 m, y 0.80 m, yaw 1.18 deg | pose ANEES 8.37 (expected 3, 95% interval [2.85, 3.15]) | landmark ANIS 0.33 per DOF (expected 1, 95% interval [0.98, 1.02])
```

The numbers are the ones the figures show. The averages follow the book's
(4.67): the pose ANEES is the mean NEES (expected 3, the pose dimension) and on
Victoria Park the GPS ANIS is the mean NIS (expected 2). The landmark NIS of a
scan has two degrees of freedom per associated landmark, which changes from
scan to scan, so it is averaged per degree of freedom instead (expected 1).
Every average comes with its 95% interval, and every per-step bound in the
figures is a 95% bound. The map-quality line needs ground truth, so it only
appears on the simulated set.

## Debugging

On the simulated set, the map-quality line (also in the title of
`final_snapshot.pdf`) is the first thing to look at:

```text
Map quality: 78 of 78 observed true landmarks mapped, 0 missed, 0 duplicates, 0 spurious
```

Duplicates mean data association failed: a measurement of a landmark already in
the map was not matched to it and became a new landmark. If your error is large
and there are no duplicates, look at the models and the noise; if there are
duplicates, look at association first.


# TTK4250 Sensor Fusion, Assignment 2: Factor Graph SLAM
 
Factor graph SLAM on simulated data and on the Victoria Park dataset, built on [GTSAM](https://gtsam.org), the library referred to in Sec. 9.5 of the book.
 
## Contents
 
- [About the template code](#about-the-template-code)
- [Setup](#setup)
- [Running](#running)
- [Repository layout](#repository-layout)
- [Task 1: Implement factor graph SLAM](#task-1-implement-factor-graph-slam)
- [Task 2: Simulated data](#task-2-simulated-data)
- [Task 3: Victoria Park dataset](#task-3-victoria-park-dataset)
- [Task 4: One bad association](#task-4-one-bad-association)
- [References](#references)
## About the template code
 
The template code is a trimmed-down factor graph SLAM system built on GTSAM.
 
- **Back-end (GTSAM, given):** holds the factor graph, maintains the Bayes tree, solves for the MAP estimate incrementally with iSAM2, and recovers marginal covariances from it.
- **Front-end (yours):** turns odometry and lidar data into factors, predicts the measurements and their innovation covariance, decides which measurement belongs to which landmark, and decides when a new landmark is born.
Data association itself (JCBB) is given. The factors and the quantities it relies on are implemented in Task 1.
 
## Setup
 
From the handout folder:
 
```bash
conda env create -f environment.yml
conda activate ttk4250_ga2
pip install -e . --no-deps
```
 
Setup problems, especially on Windows, are covered in [`docs/INSTALL.md`](docs/INSTALL.md).
 
## Running
 
| Command | Purpose |
|---|---|
| `pytest` | Run all tests, including the end-to-end tests |
| `pytest -m <part>` | Run the tests for one part of Task 1, e.g. `pytest -m b` or `pytest -m c2` |
| `run_sim` | Run on the simulated dataset |
| `run_real` | Run on the Victoria Park dataset |
| `plot_run <run directory>` | Re-plot an existing run |
 
Useful flags:
 
- `--config <file>`: use a tuning file from `configs/`
- `--output-dir <dir>`: where the run is saved
- `--no-show-plots`: skip the plot windows
- `--steps N` (`run_real`): only process the first `N` laser scans
- `--no-save-on-abort`: exit on `Ctrl+C` without saving
- `-x` (`pytest`): stop at the first failure
`Ctrl+C` stops a run and still saves it up to the last completed step, unless `--no-save-on-abort` is given.
 
Each run directory keeps the figures, the config used and the summary (in `metadata.json`).
 
## Repository layout
 
| Path | Contents | Your work |
|---|---|---|
| `configs/` | Run configurations | Everything marked `# TODO tune` (Tasks 2 and 3) |
| `docs/GTSAM.md` | Every GTSAM call Task 1 needs, with what each one means | Read it before you start |
| `preprocessing.py` | Odometry and lidar front-end. The lidar part at the bottom, which turns a raw scan into range-bearing measurements, is given. | `relative_pose`, `preintegrate` |
| `factor_graph.py` | Pose prediction, odometry and landmark factors, measurement model and its inverse, joint covariance from GTSAM's marginals, innovation covariance | Most of Task 1 |
| `landmark_manager.py` | Landmark birth | The M of N rule |
| `data_association.py` | JCBB, and the forced wrong association for Task 4 | Nothing |
| `evaluation.py` | NEES, NIS, their bounds, the run summary, and (simulated data) the comparison of the map with the true landmarks | Nothing |
| `slam.py` | Main loop tying everything together. Reading it is the fastest way to understand one time step. | Nothing |
| `loaders/`, `logger.py`, `plotter.py`, `plotting/` | Dataset adapters, run logging, plotting | Nothing |
 
## Task 1: Implement factor graph SLAM
 
A factor graph SLAM system turns sensor data into a graph and solves it. The back-end (iSAM2 through GTSAM) optimizes the graph and recovers covariances from it. Everything around it is implemented here:
 
- turning wheel odometry into relative poses (a, b),
- adding the factors (c),
- predicting each measurement and how uncertain that prediction is (d to f), which is what the given JCBB needs to decide which measurement belongs to which landmark,
- placing and confirming new landmarks (g, h).
All functions are marked with `# TODO` in the code. The GTSAM calls you need are listed in [`docs/GTSAM.md`](docs/GTSAM.md), and your editor shows their signatures.
 
### Checklist
 
Each part has its own tests, selected by a pytest marker named after the part. Use them to check each part as you go.
 
| Part | Function | Tests |
|---|---|---|
| (a) | `preprocessing.relative_pose` | `pytest -m a` |
| (b) | `preprocessing.preintegrate` | `pytest -m b` |
| (c1) | `factor_graph.predict_pose` | `pytest -m c1` |
| (c2) | `factor_graph.add_odometry_factor` | `pytest -m c2` |
| (c3) | `factor_graph.add_landmark_factor` | `pytest -m c3` |
| (d) | `factor_graph.predict_measurement` | `pytest -m d` |
| (e) | `factor_graph.assemble_joint_covariance` | `pytest -m e` |
| (f) | `factor_graph.innovation_covariance` | `pytest -m f` |
| (g) | `factor_graph.inverse_measurement` | `pytest -m g` |
| (h) | `landmark_manager.TentativeLandmark.is_confirmed` | `pytest -m h` |
 
Plain `pytest` runs everything, including the end-to-end tests, which need all parts.
 
### (a) Relative pose from wheel odometry
 
`preprocessing.relative_pose` turns one wheel-encoder reading into a relative pose increment.
 
The Victoria Park vehicle reports a forward speed $v_e$ measured at the left rear wheel and a steering angle $\alpha$. The encoder sits a distance $H$ off the centre line, so during a turn it travels along a different arc than the centre of the rear axle. With $L$ the wheelbase, the distance between the axles, the kinematic bicycle model gives the body twist
 
$$
v_b = \frac{v_e}{1 - \frac{H}{L}\tan\alpha}, \qquad
\omega = \frac{v_b}{L}\tan\alpha, \qquad
\xi = \begin{bmatrix} v_b & 0 & \omega \end{bmatrix}^T.
$$
 
Held constant over the time $\Delta t$ between two odometry readings, this twist integrates exactly to $\Delta T = \operatorname{Exp}(\xi \Delta t)$, which follows a circular arc.
 
> **Hint:** The exponential map is covered in Sec. 6.2.3. `gtsam.Pose2.Expmap` implements it, taking $\xi \Delta t$ as a numpy array. The vehicle geometry is the `car` argument: $H$ is `car.H` and $L$ is `car.L`.
 
### (b) Preintegration
 
`preprocessing.preintegrate` compounds a sequence of relative poses into a single one, with corresponding covariance.
 
Wheel odometry arrives far faster than lidar scans. Rather than putting a pose in the graph for every encoder reading, all increments between two scans are compounded into one relative pose and one covariance, which is added to the graph as a single `BetweenFactorPose2`. This is called preintegration (Sec. 9.1).
 
The mean is repeated composition. The covariance is propagated through the Jacobians of each composition,
 
$$
\Sigma \leftarrow H_1 \Sigma H_1^T + H_2 \Sigma_i H_2^T,
$$
 
where $\Sigma_i$ is the covariance of increment $i$, and $H_1$ and $H_2$ are the Jacobians of `compose` with respect to its first and second argument.
 
> **Hint:** Start the recursion from the identity pose `gtsam.Pose2()` and a zero $3 \times 3$ covariance. `compose` fills in both Jacobians for you if you pass it two arrays preallocated as `np.zeros((3, 3), order="F")`.
 
### (c) Build the graph
 
Each factor you add is one term of the negative log posterior (9.4).
 
#### (c1) Predict the pose
 
`factor_graph.predict_pose` returns the predicted pose
 
$$
\hat{x}_k = \hat{x}_{k-1} \oplus u_k,
$$
 
the previous pose estimate compounded with the odometry increment $u_k$, which is expressed in the body frame of $x_{k-1}$. For a `gtsam.Pose2`, $\oplus$ is `compose`.
 
Unlike the prediction step of an EKF, this prediction does not alter the estimate. It only gives the optimizer a starting guess for the new pose $x_k$, which an iterative solver needs for every variable. What connects $x_k$ to $x_{k-1}$ is the odometry factor added in (c2).
 
#### (c2) Odometry factor
 
`factor_graph.add_odometry_factor` adds the middle term of (9.6) as a `BetweenFactorPose2`:
 
```python
factor = gtsam.BetweenFactorPose2(key_previous, key_current, measured, noise_model)
graph.add(factor)
```
 
> **Hint:** Use `gtsam.noiseModel.Gaussian.Covariance`, which takes the full $3 \times 3$ matrix, and not `.Sigmas`, which takes only standard deviations. The covariance preintegrated in (b) is not diagonal. A heading error also moves the pose sideways, so the two are correlated. Dropping that correlation makes the factor too confident in some directions and not confident enough in others.
 
#### (c3) Landmark factor
 
`factor_graph.add_landmark_factor` adds the last term of (9.6) as a `BearingRangeFactor2D`:
 
```python
factor = gtsam.BearingRangeFactor2D(pose_key, landmark_key, bearing, range, noise_model)
```
 
> **Hint:** The bearing comes first, as a `gtsam.Rot2(angle)`, and the range as a float. A plain float bearing raises a `TypeError`. The noise model you are handed is already ordered `[bearing, range]` to match.
 
### (d) Predict the measurement
 
`factor_graph.predict_measurement` implements the measurement function $h(x_k, m_i)$ of (9.6), together with the two blocks of the Jacobian $J$ in (9.8) belonging to this pose and this landmark. It returns
 
- the predicted measurement $z = [\text{range}, \text{bearing}]$,
- the pose block $\partial z / \partial x_k$ ($2 \times 3$),
- the landmark block $\partial z / \partial m_i$ ($2 \times 2$).
You do not derive any of this yourself. GTSAM computes the values and the Jacobians one row at a time, and your task is to assemble them:
 
1. Preallocate four sub-Jacobians, one row each: range and bearing, each with respect to the pose ($1 \times 3$) and to the landmark ($1 \times 2$). Any other shape or type makes GTSAM raise a `TypeError`.
```python
   H_range_pose = np.zeros((1, 3), order="F")
```
 
2. Pass them to the two GTSAM calls. Each returns its value and fills in its two sub-Jacobians:
```python
   pose.range(landmark, H_range_pose, H_range_landmark)
   pose.bearing(landmark, H_bearing_pose, H_bearing_landmark)
```
 
   `bearing` returns a `gtsam.Rot2`, not a float. Take `.theta()` for the angle.
 
3. Stack the rows with `np.vstack`, range first and bearing second, to match the order of $z$.
### (e) Joint covariance
 
`factor_graph.assemble_joint_covariance` builds the joint covariance $P$ of the current pose and the landmarks in the local map: the $P$ in $S = HPH^T + R$ of (f).
 
A factor graph stores the sparse information matrix $\Lambda$, not the covariance. GTSAM computes $P$ from $\Lambda^{-1}$ for the variables you ask for (Sec. 9.4) and hands it to you as a `gtsam.JointMarginal`, in blocks:
 
```python
joint_marginal.at(key_i, key_j)  # covariance between variables key_i and key_j
```
 
Put these blocks together into one matrix, in the order of `keys`: the pose first, then the landmarks. Either stack the blocks with `np.block`, or preallocate $P$ and fill it block by block in two loops over `keys`. For the loops, the size of each variable's block is `joint_marginal.at(key, key).shape[0]`.
 
> **Important:** Include every block, also the ones pairing two different landmarks, $P_{m_i m_j}$. Leaving them out gives a plausible-looking but wrong $S$. Also keep the order of `keys`, since (f) stacks the Jacobians in that order.
 
### (f) Innovation covariance
 
`factor_graph.innovation_covariance` assembles
 
$$
S = HPH^T + R
$$
 
for the whole local map at once. This is (9.26) with $P$ the joint covariance from (e), $H$ the stacked Jacobian $J_a$ from (d), and $R$ the block-diagonal measurement noise $R_a$.
 
With `n = len(jacobians_pose)` the number of landmarks in the local map, the dimensions are:
 
- $P$ is $(3+2n) \times (3+2n)$, pose first and then each landmark,
- $H$ is $2n \times (3+2n)$, two rows per predicted measurement,
- $R$ and the returned $S$ are $2n \times 2n$.
For $n = 3$,
 
$$
H =
\begin{array}{c|cccc}
 & x_k & m_0 & m_1 & m_2 \\
\hline
z_0 & H^x_0 & H^m_0 & 0 & 0 \\
z_1 & H^x_1 & 0 & H^m_1 & 0 \\
z_2 & H^x_2 & 0 & 0 & H^m_2
\end{array}
$$
 
where $H^x_i$ is `jacobians_pose[i]` ($2 \times 3$) and $H^m_i$ is `jacobians_landmark[i]` ($2 \times 2$). $H$ is dense in its three leftmost columns, because every predicted measurement depends on the pose, and block diagonal elsewhere, because predicted measurement $i$ depends only on landmark $i$. $R$ is `measurement_cov` repeated $n$ times along its diagonal, since the measurements are independent.
 
### (g) Inverse measurement
 
`factor_graph.inverse_measurement` implements the inverse measurement function $h^{-1}(z, x)$: given a measurement $z$ taken from pose $x$, it returns the landmark's position in the world frame. It places a new landmark for a measurement that JCBB could not associate. Such a measurement may also be clutter, so (h) decides when the new landmark enters the graph.
 
Unlike in EKF-SLAM (Sec. 7.2), only the position is needed, not a covariance. Once the landmark is connected by factors, its covariance follows from the graph, like that of every other variable.
 
> **Hint:** Find the landmark in the body frame first. `rotate` needs a point, not a range, so start from `gtsam.Point2(range, 0.0)`, the landmark straight ahead, and let `gtsam.Rot2(bearing).rotate(...)` turn it to the measured bearing, giving $(r \cos b, r \sin b)$. Then `pose.transformFrom(...)` takes that point from the body frame to the world frame.
 
### (h) M of N confirmation
 
`landmark_manager.TentativeLandmark.is_confirmed` implements the M of N confirmation rule from target tracking: confirm a tentative landmark once it has been seen in at least $M$ of the last $N$ time steps, the current one included.
 
The evidence is the list `self.supporting_observations`, with one entry per time step the landmark was seen in. The given code fills it, starting with the measurement that created the landmark. Each entry's time step is in `.step`. Your function only reads the list: count the entries whose `.step` lies inside the window, and compare with $M$.
 
> **Hint:** Both ends are easy to get off by one. With $N = 3$ and `current_step` 10, the window is steps 8, 9 and 10: it starts at $10 - 3 + 1$, and a step equal to the start counts, so compare with `>=`. Exactly $M$ hits is enough, so compare the count with `>=` as well.
 
## Task 2: Simulated data
 
### Dataset
 
| Field | Shape | Description |
|---|---|---|
| `odometry` | `(K, 3)` | Measured movement of the robot in its body frame |
| `z` | `(K,)` | List of detections at each time step, each of shape `(number of measurements, 2)` |
| `poseGT` | `(K, 3)` | True pose at each time step |
| `landmarks` | `(L, 2)` | True landmark positions |
 
### Baseline
 
The initial tuning is in `configs/sim_default.yaml`. It is a working baseline: the run completes and the map is mostly right, but there is room for improvement in both accuracy and consistency. The pose NEES sits above its expected value, and the landmark NIS below it.
 
With a correct Task 1, the baseline gives:
 
- 72 of the 78 true landmarks mapped,
- 3 spurious landmarks,
- pose ANEES of 8.37.
The run is deterministic, so if yours differs, check Task 1 before you start tuning.
 
### Running
 
```bash
run_sim --output-dir runs/sim/baseline
```
 
For each tuning set, copy `configs/sim_default.yaml` to a new file in `configs/`, change its values, and run it with its own config and output directory:
 
```bash
run_sim --config configs/<my_tuning>.yaml --output-dir runs/sim/<my_tuning>
```
 
A run takes about 10 to 15 seconds.
 
### Deliverable
 
Find 3 sets of tuning values that differ from the baseline in different ways, and explain the results you get with them. For example:
 
- a set that performs better than the baseline in some way,
- a set with similar errors but poor consistency,
- a set that maps noticeably fewer landmarks, and how that affects the trajectory.
### Judging consistency
 
Every run prints a summary of the averaged NEES and NIS (the landmark ANIS is per degree of freedom, since the number of associated landmarks varies). To judge consistency:
 
- compare each average with its expected value,
- check that about 95% of the per-step values fall inside their bounds.
Do not rely on the 95% interval of the average alone. Equation (4.67) assumes independent errors, but the errors of neighbouring poses in a smoother are strongly correlated, so even with the correct noise, one run's ANEES can land anywhere between about 1.5 and 8.
 
### Map quality
 
Every simulated run also prints a map-quality summary comparing the map with the true landmarks. Each map landmark is paired with at most one true landmark, closest pairs first and only within 3 m, and only the true landmarks the run actually observed are counted.
 
| Category | Meaning |
|---|---|
| **Mapped** | A true landmark with a map landmark paired to it. The RMS position error is taken over these pairs. |
| **Missed** | An observed true landmark with no map landmark paired to it. Either it was never confirmed (not seen in M of N steps, or its measurements kept being associated elsewhere), or its estimate drifted more than 3 m away. |
| **Duplicate** | An extra map landmark within 3 m of a true landmark that already has its pair. Data association failed to match a measurement to the landmark already in the map, and a new landmark was born from it instead. |
| **Spurious** | A map landmark with no true landmark within 3 m. It was born from a wrong measurement or a wrong association, or its estimate has drifted far from where the landmark is. |
 
Duplicates point at the front-end (association and landmark birth). Missed and spurious landmarks can come from either end.
 
> **Note:** The runtime of JCBB grows quickly with the number of duplicate landmarks, because each one gives a measurement more candidates to pair with. A tuning that creates duplicates (noise values so small that the gates reject true re-observations, or landmark birth on a single sighting, $M = N = 1$, with a wrong noise model) can slow a run from seconds to minutes. If a run slows down substantially, stop it with `Ctrl+C`. The run so far is saved, and its map-quality summary shows whether duplicates are the cause.
 
### Discussion questions (optional)
 
- Which types of errors do you see, and why do you think they look like this?
- How much of your error is the front-end responsible for, and how much the back-end (Sec. 9.1)? Use the map-quality numbers.
- Would you say that there are any shortcomings of the algorithm on this dataset?
## Task 3: Victoria Park dataset
 
Victoria Park is a real dataset: a vehicle with a laser scanner, which sees the tree trunks, drives through a park, with a GPS for reference (see the [dataset's web page](#references)). The loader converts all times to seconds and turns the raw data into odometry and tree detections.
 
### Raw data
 
| Field | Shape | Description |
|---|---|---|
| `LASER` | `(mK, 361)` | Laser scans, 361 returns spread over a half circle |
| `speed` | `(K,)` | Speed from a wheel encoder at the rear left wheel |
| `steering` | `(K,)` | Steering angle |
| `Lo_m`, `La_m` | `(Kgps,)` | GPS position in metres |
| `timeLsr`, `timeOdo`, `timeGps` | | Time stamps of the scans, the odometry and the GPS, in milliseconds |
 
### Baseline
 
The initial tuning is in `configs/real_default.yaml`, and it already works well: the map does not fill up with duplicate landmarks, and the trajectory stays close to the GPS. Consistency, however, can be improved. The landmark NIS sits well below its expected value, and improving this is the main point of this task.
 
### Deliverable
 
As in Task 2, find 3 sets of tuning values that differ in different ways, and explain the results you get with them.
 
- Do not expect to get much closer to the GPS than the baseline. A better set is one that improves consistency without giving up what the baseline does well.
- At least one set should lower the odometry noise.
- Some sets will break the run: the map fills up with duplicate landmarks and the trajectory drifts away from the GPS. When that happens, describe what breaks and try to explain why it happens.
### Planning your runs
 
A full run over all 7247 laser scans (the default) took about 2.5 minutes with the given tuning on the course staff's machine, and about twice that when the map grows. Expect it to take longer on a laptop. To find good values without hours of waiting:
 
- **Explore with `run_real --steps 2000`**, which takes about 20 seconds. The first 2000 scans cover most of the park, including the first loop closures: places where the car returns to trees it has mapped before, so the graph connects its current pose to landmarks seen earlier. A tuning that breaks in this first period will not recover, but one that looks good can still fall apart later, so confirm a promising set with a full run.
- **Change one group of parameters at a time** (odometry noise, measurement noise, gates, landmark birth), so that you know what caused what.
- **Watch the number of landmarks** printed at the end of each run. The given tuning ends the first 2000 scans with about 140. If several hundred are observed, data association is likely failing.
All figures and numbers in the report should come from full runs.
 
### About the GPS reference
 
There is no ground truth, so the plots compare with the GPS instead.
 
- The GPS is far from perfect, and it drops out for about 40% of the drive, so the GPS NIS and the distance to the GPS only cover the times it is available.
- The GPS is not used by the SLAM system. It only serves as a reference.
- The GPS NIS assumes the GPS has standard deviation `noise.sigma_gnss` (1 m by default). This value is a guess about the GPS. Changing it does not change the estimate, only the GPS NIS, and a larger value gives a lower GPS NIS for the same estimate. **Do not tune it to improve the plot.**
### Discussion questions (optional)
 
- Which types of errors do you see, and how do they differ from those on the simulated data? Why?
- What happens to the estimate of the whole trajectory when a loop is closed?
- What shortcomings does the algorithm show on this dataset, and how could it be improved?
## Task 4: One bad association
 
This short task looks at what happens when a data association is wrong, and how a robust loss function can help. The experiment is quick to run, and its discussion should fit in one figure and a few short paragraphs.
 
### Injecting a wrong association
 
`configs/task4.yaml` sets `inject_wrong_at_step: 600` under `association`, which re-pairs one measurement with the wrong landmark at step 600. Everything else is the same as in `configs/sim_default.yaml`. Run it with
 
```bash
run_sim --config configs/task4.yaml --output-dir runs/sim/task4
```
 
and compare it with your Task 2 baseline:
 
- Explain what this single wrong association does to the trajectory and the map, using the error values and landmark counts from the run summary.
- Besides the usual figures, this run saves `wrong_association.pdf`, which shows the landmark NIS on a log scale and the number of associated measurements around step 600. How does the landmark NIS behave at step 600 and in the steps after it, and why?
### Huber loss
 
The Huber loss is a robust loss function (GTSAM calls it a robust kernel, hence the config name) that is quadratic for small errors and linear for large ones:
 
$$
\rho_\delta(e) =
\begin{cases}
e^2, & |e| \le \delta, \\
2\delta|e| - \delta^2, & |e| > \delta.
\end{cases}
$$
 
With `backend.robust_kernel: huber`, each landmark term in (9.6) changes from
 
$$
\left\| h(x_k, m_i) - z_k^i \right\|_R^2
\quad \text{to} \quad
\rho_\delta\left( \left\| h(x_k, m_i) - z_k^i \right\|_R \right),
$$
 
where $\| h(x_k, m_i) - z_k^i \|_R$ is the measurement error normalized by the measurement noise covariance $R$ (its Mahalanobis distance). The threshold $\delta$, set by `backend.huber_k`, is therefore in units of measurement noise standard deviations. The Wikipedia article on the Huber loss defines it with an extra factor $\frac{1}{2}$.
 
Edit `configs/task4.yaml`:
 
```yaml
backend:
  robust_kernel: huber
```
 
and run again with
 
```bash
run_sim --config configs/task4.yaml --output-dir runs/sim/task4_huber
```
 
### Deliverable
 
- Explain how this changes the cost function (9.6) for the wrong measurement, and why that helps. The two runs' `wrong_association` figures show the difference.
- Compare with EKF-SLAM (Sec. 7.2): what are your options for recovering from a wrong association in each of the two frameworks?
## References
 
1. Victoria Park dataset: <http://www-personal.acfr.usyd.edu.au/nebot/victoria_park.htm>
2. Huber loss: <https://en.wikipedia.org/wiki/Huber_loss>
 


