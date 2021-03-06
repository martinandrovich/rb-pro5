# Changelog

All notable changes to this project will be documented in this file; a full list of changes can be found at the [issues] page. This project adheres to [Semantic Versioning] and the changelog format is based on both [keep-a-changelog] and [github-changelog-generator].

<!-- ------------------------------------------------------------------------------------------------------------------------------ -->

## [Unreleased]

#### Milestones

- Implement dual fuzzy controller for target navigation w/ obstacle avoidance [\#M1]

#### Suggestions

- Upgrade to OpenCV4 [\#10]

#### Todos

- Path generator
- Adjacency graph with coverage methods [\#17]
- Marble detection (OpenCV)
- Localization (particle filter)
- Make all custom types atomic with get() and get() methods [\#3]
- Document fuzzy controller

#### Features and enhancements

- Add utilities module in `utils.h` with some helper methods
- Add dimension type `dim_t`
- Update constructors of `pos_t` and `dim_t` to work with `std::initializer_list<float>`
- Obstacle avoidance fuzzy controller `core::flctrl_obs_avoid()` [\#11]
- Goal navigation fuzzy controller `core::flctr_goal_nav()` [\#8]
- Add dumplog to `debug.h`

#### Bugfixes

- Item.

#### Closed issues

- Item.

<!-- ------------------------------------------------------------------------------------------------------------------------------ -->

## [0.2.0] - 2019-10-22
Refactor to new `core.h`, implement custom types in `modules/types/` (mostly atomic), add `debug.h` window + other minor improvements. Setup fuzzy control of goal navigation and local obstacle avoidance, located in `flctrl.h`.

#### Features and enhancements

- Merge and refactor to `main.new.cpp` using `core.h` [\#6]
- Move all fuzzy control to separate file `flctrl.h` [\#14]
- Move all custom types into `modules/types/` with each type having own header [\#2]
- Fuzzy controller selector `core::flctrl()` [\#7]
- Velocity command publisher `core::publish_velcmd()`
- Rename of vel_t members [\#4] ([slaxzer])
- Debug window [\#5] ([androvich])
- Add issues and milestones to GitHub + update changelog [\#1] ([androvich])

<!-- ------------------------------------------------------------------------------------------------------------------------------ -->

## [0.1.0] - 2019-10-01
Setup the basic in-development project. Basic types defined in `core-common.h` and core functionality is defined in `core.h`, which can be initilazed from `main.cpp`.

#### Features and enhancements

- Add `pos_t` type
- Add `orient_t` type
- Add `vel_t` type
- Add `pose_t` type
- Add `obs_t` type
- Add `lidar_t` type
- Add `camera_t` type
</br></br>
- Implement open world obstacle avoidance in `main.cpp` [deprecated]
- Implement `core::init()`
- Implement `core::run()`
- Implement gazebo callbacks; lidar, camera, pose

<!-- ------------------------------------------------------------------------------------------------------------------------------ -->

## [0.0.0] - 2019-10-01 (Example)
A small paragraph about the major changes in this version could be written here; it should simply summarize the release and highlight any important issues.

#### Features and enhancements

- Add `lidar_t` type [\#19] ([slaxzer])

#### Bugfixes

- out of bounds exception in `core::init()` [\#14] ([daniel])

#### Closed issues

- How can be better structure the code? [\#0] ([androvich])

<!-- Links ------------------------------------------------------------------------------------------------------------------------ -->

<!-- -- External ------------------------------------------------------------------------------------------------------------------ -->

[Semantic Versioning]: https://semver.org/spec/v2.0.0.html
[keep-a-changelog]: https://github.com/olivierlacan/keep-a-changelog
[github-changelog-generator]: https://github.com/github-changelog-generator/github-changelog-generator
[issues]: https://github.com/martinandrovich/rb-pro5/issues

<!-- -- Releases ------------------------------------------------------------------------------------------------------------------ -->

[0.2.0]: https://github.com/martinandrovich/rb-pro5/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/martinandrovich/rb-pro5/releases/tag/v0.1.0
[0.0.0]: #changelog

<!-- -- Milestones----------------------------------------------------------------------------------------------------------------- -->

[\#M1]:  https://github.com/martinandrovich/rb-pro5/milestone/1

<!-- -- Issues -------------------------------------------------------------------------------------------------------------------- -->

[\#19]:  https://github.com/github-changelog-generator/github-changelog-generator/issues/19
[\#17]:  https://github.com/martinandrovich/rb-pro5/issues/17
[\#14]:  https://github.com/martinandrovich/rb-pro5/issues/14
[\#11]:  https://github.com/martinandrovich/rb-pro5/issues/11
[\#10]:  https://github.com/martinandrovich/rb-pro5/issues/10
[\#8]:   https://github.com/martinandrovich/rb-pro5/issues/8
[\#7]:   https://github.com/martinandrovich/rb-pro5/issues/7
[\#6]:   https://github.com/martinandrovich/rb-pro5/issues/6
[\#5]:   https://github.com/martinandrovich/rb-pro5/issues/5
[\#4]:   https://github.com/martinandrovich/rb-pro5/issues/4
[\#3]:   https://github.com/martinandrovich/rb-pro5/issues/3
[\#2]:   https://github.com/martinandrovich/rb-pro5/issues/2
[\#1]:   https://github.com/martinandrovich/rb-pro5/issues/1
[\#0]:   https://github.com/github-changelog-generator/github-changelog-generator/issues/19

<!-- -- Identities ---------------------------------------------------------------------------------------------------------------- -->

[androvich]: https:/github.com/martinandrovich
[daniel]: https://github.com/dscho15
[slaxzer]: https://github.com/slaxzer96
