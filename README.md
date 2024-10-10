- [OpenDTU-OnBattery](#opendtu-onbattery)
  - [What is OpenDTU-OnBattery](#what-is-opendtu-onbattery)
  - [Documentation](#documentation)
  - [State of the project](#state-of-the-project)
  - [History of the project](#history-of-the-project)
  - [Acknowledgments](#acknowledgments)

# OpenDTU-OnBattery

This is a fork of [OpenDTU](https://github.com/tbnobody/OpenDTU).

<!---
disabled while "create release badge" action is broken, see .github/build.yml
![GitHub tag (latest SemVer)](https://img.shields.io/endpoint?url=https://gist.githubusercontent.com/helgeerbe/68b47cc8c8994d04ab3a4fa9d8aee5e6/raw/openDTUcoreRelease.json)
--->

[![OpenDTU-OnBattery Build](https://github.com/hoylabs/OpenDTU-OnBattery/actions/workflows/build.yml/badge.svg)](https://github.com/hoylabs/OpenDTU-OnBattery/actions/workflows/build.yml)
[![cpplint](https://github.com/hoylabs/OpenDTU-OnBattery/actions/workflows/cpplint.yml/badge.svg)](https://github.com/hoylabs/OpenDTU-OnBattery/actions/workflows/cpplint.yml)
[![Yarn Linting](https://github.com/hoylabs/OpenDTU-OnBattery/actions/workflows/yarnlint.yml/badge.svg)](https://github.com/hoylabs/OpenDTU-OnBattery/actions/workflows/yarnlint.yml)

## What is OpenDTU-OnBattery

OpenDTU-OnBattery is an extension of the original OpenDTU to support battery
chargers, battery management systems (BMS) and power meters on a single ESP32.
With the help of a Dynamic Power Limiter, the power production can be adjusted
to the actual consumption. In this way, it is possible to implement a zero
export policy.

## Documentation

The canonical documentation of OpenDTU-OnBattery is hosted at
[https://opendtu-onbattery.net](https://opendtu-onbattery.net).

You may find additional helpful information in the project's
community-maintained [Github
Wiki](https://github.com/hoylabs/OpenDTU-OnBattery/wiki).

To find out what's new or improved have a look at the changelog of the
[releases](https://github.com/hoylabs/OpenDTU-OnBattery/releases).

## State of the project

OpenDTU-OnBattery is actively maintained. Please note that OpenDTU-OnBattery
may change significantly during its development. Bug reports, comments, feature
requests and pull requests are welcome!

## History of the project

The original OpenDTU project was started from [a discussion on
Mikrocontroller.net](https://www.mikrocontroller.net/topic/525778). It was the
goal to replace the original Hoymiles DTU (Telemetry Gateway) to avoid using
Hoymile's cloud. With a lot of reverse engineering the Hoymiles protocol was
decrypted and analyzed.

In the summer of 2022 @helgeerbe bought a Victron MPPT charge cntroller, and
didn't like the idea to set up a separate ESP32 to receive the charger's data.
He decided to fork OpenDTU and extend it with battery charger support and a
Dynamic Power Limiter.

In early October 2024, the project moved to the newly founded GitHub
organisation `hoylabs` and is since maintained by multiple community members.

## Acknowledgments

* Special thanks to Thomas Basler (@tbnobody), the author of the [upstream
  project](https://github.com/tbnobody/OpenDTU), for hist continued effort!
* Thanks to @helgeerbe for starting OpenDTU-OnBattery and his dedication to the
  project, as well as his trust in the current maintainers of the project,
  which act as part of the `hoylabs` GitHub organisation.
* We like to thank all contributors. With your ideas and enhancements, you have
  made OpenDTU-OnBattery much more than @helgeerbe originally had in mind.
