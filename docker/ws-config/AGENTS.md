# Project Guidelines

## Code Style

> These styles only applied for new implementation. Do not apply these on existed code.

- Primary language in this workspace is C++ for ROS 2 controllers.
- Follow formatting in src/inin_controllers/.clang-format:
  - Google-based style
  - 2-space indentation
  - 100-column limit
  - C++17-oriented conventions
- For spatial states, use the following strict lowercase naming pattern:
  - Format: `quantity_frame1_frame2` (ALL LOWERCASE).
  - Frames: Use shorthand prefixes: `w` (world), `e` (end-effector), `vir` (virtual), `ref` (reference), `cur` (current).
  - Quantities: Use suffixes like `pose`, `trans`, `rot`, `twist`, `vel`, `omega`, `wrench`, `force`, `torque`.
  - Semantics: `pose_a_b` means the pose of frame `b` measured in frame `a`. It also represents the transformation matrix from frame `b` to frame `a` (i.e., $^aT_b$).
  - If only one frame is provided (e.g., `vel_cur`), assume the quantity is measured in the body frame (Local Frame).
- Do not perform any git operations except asked by user.

## Environment Notes

- Default colcon behavior is also documented in:
  - colcon_defaults.yaml
- DDS is configured for rmw_zenoh_cpp in the container image.
- Do not run any program that involved real hardwares, except asked by user

## Real-Time C++ & Eigen Coding Rules

- Absolute Zero-Allocation (Strict Eigen Memory Rules)
  - **Prohibited:** `Eigen::MatrixXd`, `VectorXd`, or any dynamic-sized algorithms or apis inside the real-time loop.
  - **Mandate:** Force stack allocation for dimensions known at compile-time (e.g., use `Eigen::Matrix<double, 6, 1>` instead of dynamic vectors).
- The `.noalias()` Trap & Matrix Chaining**
  - **Prohibited:** Chaining 3 or more matrices in a single expression (e.g., `res = A * B * C`). This triggers hidden heap allocation (`malloc`), even with `.noalias()`.
  - **Mandate:** Decompose complex expressions. Break them into pairs using pre-allocated class-member buffers (e.g., `buffer.noalias() = A * B; res.noalias() = buffer * C;`).
- Diagonal & Identity Matrix Optimization**
  - **Prohibited:** Instantiating full $N \times N$ matrices for diagonals or identities (e.g., `Matrix::Identity() * scalar`).
  - **Mandate:** Use `V.asDiagonal()` for scaling operations. Use `.diagonal().array() += scalar` to add to an existing matrix's diagonal in-place ($O(N)$ vs $O(N^2)$).
- Algebraic Pre-Simplification**
  - **Mandate:** Mentally simplify math equations *before* generating C++ code. Eliminate redundant matrix multiplications (e.g., $R \cdot R^T = I$) and reduce spatial transformations to their simplest vector addition/subtraction forms where mathematically equivalent.
- Data exchanges between real-time and un-real-time threads
  - **Prohibited:** Any operations that require locks or lead to race conditions.
  - **Mandate:** `realtime_tools::RealtimeBuffer` or `realtime_tools::RealtimeBox` MUST be used for lock-free data exchanges between non-real-time callbacks and the real-time loop.
