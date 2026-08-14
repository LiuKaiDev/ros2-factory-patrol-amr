Do not rewrite existing architecture unless necessary.

Do not bypass cmd_vel mux or safety gate.

Do not publish robot velocity directly from perception.

Do not replace Nav2.

Do not introduce SLAM/VIO/3D detection unless explicitly requested.

Do not fabricate runtime results.

Keep perception modular and replaceable.

Preserve existing validation scripts and tests.