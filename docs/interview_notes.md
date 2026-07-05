# Interview Notes

Use this page to explain the project without overstating what has been proven.
The recommended story is: scenario -> closed loop -> algorithms -> engineering
implementation -> safety -> validation -> boundaries.

## 30-Second Version

This is a ROS2 Jazzy low-speed patrol AMR navigation-control project. It builds
a complete engineering loop from patrol goals through AMCL, Nav2 planning and
control, velocity muxing, a safety gate, chassis adapters, odometry/TF feedback,
and health monitoring. The project uses Gazebo/RViz and mock backends for
repeatable demos, with real experiment metrics left as `TBD` until measured.

## 2-Minute Version

The target scenario is a semi-closed patrol robot in corridors, labs, parks, or
factory aisles. The robot follows patrol goals on a static map, localizes with
AMCL, plans with Nav2, and sends candidate velocity commands through a mux and
final safety gate before reaching the chassis adapter.

The repository separates the system into navigation, localization, control,
chassis, safety, simulation, and experiment packages. It includes Nav2 basic and
advanced parameter sets, an RViz debug layout, factory patrol simulation assets,
mock/serial/UDP chassis backends, safety state topics, localization health
monitoring, and scripts for static and runtime validation.

The important boundary is that simulation and mock checks are not presented as
field deployment results. Metrics in the experiment report stay `TBD` until real
logs, commands, commit IDs, maps, and parameter files are recorded.

## 5-Minute Structure

1. Scenario: low-speed patrol in a semi-closed environment.
2. Closed loop: goal -> localization -> planning -> control -> safety -> chassis
   -> odom/TF/state feedback.
3. Algorithms: AMCL, Navfn/SmacPlanner2D, RPP/MPPI, Pure Pursuit/Stanley
   standalone tracking.
4. Engineering: ROS2 packages, launch files, parameters, mock/serial/UDP
   backends, validation scripts.
5. Safety: final `/cmd_vel` gate, emergency stop, watchdog, speed limit,
   localization and chassis health inputs.
6. Validation: static checks, runtime topic checks, report templates, showcase
   placeholders.
7. Boundary: no fake metrics, no production hardware claim, real tuning remains
   future work.

## Common Questions

| Question | Answer points |
| --- | --- |
| Why AMCL? | The project targets mapped 2D semi-closed scenes with laser input. AMCL is mature, explainable, and fits this scope. |
| Why RPP and MPPI? | RPP is a clear low-speed path-following controller for the basic Nav2 setup. MPPI is kept as an advanced controller option. |
| Why Pure Pursuit and Stanley? | They are standalone tracking experiments used for controller comparison, not replacements for Nav2 plugins. |
| How is `/cmd_vel` protected? | Candidate commands pass through muxing and a final safety gate that can stop or limit commands based on emergency stop, watchdog, manual takeover, localization, scan, odom, and chassis state inputs. |
| Is real hardware integrated? | The repository has mock, serial, and UDP adapter layers. Real hardware calibration and long-run field results are still future work. |
| Are there experiment results? | Tooling and templates exist. Numeric results remain `TBD` unless produced by a real logged run. |

## Phrases To Avoid

- "Production-ready autonomous vehicle."
- "Proven in a real factory" unless the specific run evidence is available.
- "Guaranteed safety" or "certified safety."
- Any numeric performance metric that is not traceable to a real run.
