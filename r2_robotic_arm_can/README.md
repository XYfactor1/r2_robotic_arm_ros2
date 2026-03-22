# R2RoboticArm CAN Library

A C++ library for CAN communication with R2RoboticArm robotic hardware, supporting Damiao motors over CAN/CAN-FD interfaces.
This library is a part of [R2RoboticArm](https://github.com/enactic/r2_robotic_arm/). See detailed setup guide and docs [here](https://docs.r2_robotic_arm.dev/software/can).


## Quick Start

### Prerequisites

- Linux with SocketCAN support
- CAN interface hardware

### 1. Install

#### Ubuntu

* 22.04 Jammy Jellyfish
* 24.04 Noble Numbat

```bash
sudo apt install -y software-properties-common
sudo add-apt-repository -y ppa:r2_robotic_arm/main
sudo apt update
sudo apt install -y \
  libr2_robotic_arm-can-dev \
  r2_robotic_arm-can-utils
```

#### AlmaLinux, CentOS, Fedora, RHEL, and Rocky Linux

1. Enable [EPEL](https://docs.fedoraproject.org/en-US/epel/). (Not required for [Fedora](https://fedoraproject.org/))
   * AlmaLinux 8 / Rocky Linux 8
     ```bash
     sudo dnf install -y epel-release
     sudo dnf config-manager --set-enabled powertools
     ```
   * AlmaLinux 9 & 10 / Rocky Linux 9 & 10
     ```bash
     sudo dnf install -y epel-release
     sudo crb enable
     ```
   * CentOS Stream 9
     ```bash
     sudo dnf config-manager --set-enabled crb
     sudo dnf install -y https://dl.fedoraproject.org/pub/epel/epel{,-next}-release-latest-9.noarch.rpm
     ```
   * CentOS Stream 10
     ```bash
     sudo dnf config-manager --set-enabled crb
     sudo dnf install -y https://dl.fedoraproject.org/pub/epel/epel-release-latest-10.noarch.rpm
     ```
   * RHEL 8 & 9 & 10
     ```bash
     releasever="$(. /etc/os-release && echo $VERSION_ID | grep -oE '^[0-9]+')"
     sudo subscription-manager repos --enable codeready-builder-for-rhel-$releasever-$(arch)-rpms
     sudo dnf install -y https://dl.fedoraproject.org/pub/epel/epel-release-latest-$releasever.noarch.rpm
     ```
2. Install the package.
   ```bash
   sudo dnf update
   sudo dnf install -y \
     r2_robotic_arm-can-devel \
     r2_robotic_arm-can-utils
   ```

### 2. Setup CAN Interface

Configure your CAN interface using the provided script:

```bash
# CAN 2.0 (default)
r2_robotic_arm-can-configure-socketcan can0

# CAN-FD with 5Mbps data rate
r2_robotic_arm-can-configure-socketcan can0 -fd
```

### 3. C++ Library

```cpp
#include <r2_robotic_arm/can/socket/r2_robotic_arm.hpp>
#include <r2_robotic_arm/damiao_motor/dm_motor_constants.hpp>

r2_robotic_arm::can::socket::R2RoboticArm arm("can0", true);  // CAN-FD enabled
std::vector<r2_robotic_arm::damiao_motor::MotorType> motor_types = {
    r2_robotic_arm::damiao_motor::MotorType::DM4310, r2_robotic_arm::damiao_motor::MotorType::DM4310};
std::vector<uint32_t> send_can_ids = {0x01, 0x02};
std::vector<uint32_t> recv_can_ids = {0x11, 0x12};

r2_robotic_arm.init_arm_motors(motor_types, send_can_ids, recv_can_ids);
r2_robotic_arm.enable_all();
```

See [dev/README.md](dev/README.md) for how to build.

### 4. Python (🚧 EXPERIMENTAL - TEMPORARY 🚧)

> [!WARNING]
>
> ⚠️ **WARNING: UNSTABLE API** ⚠️
> Python bindings are currently a direct low level **temporary port**, and will change **DRASTICALLY**.
> The interface is may break between versions.Use at your own risk! Discussions on the interface are welcomed.

**Build & Install:**

Please ensure that you install the C++ library first, as `1. Install` or [dev/README.md](dev/README.md).

```bash
cd python

# Create and activate virtual environment (recommended)
python -m venv venv
source venv/bin/activate

pip install .
```

**Usage:**

```python
# WARNING: This API is unstable and will change!
import r2_robotic_arm_can as oa

arm = oa.R2RoboticArm("can0", True)  # CAN-FD enabled
arm.init_arm_motors([oa.MotorType.DM4310], [0x01], [0x11])
arm.enable_all()
```

### Examples

- **C++**: `examples/demo.cpp` - Complete arm control demo
- **Python**: `python/examples/example.py` - Basic Python usage

## For developers

See [dev/README.md](dev/README.md).

## Related links

- 📚 Read the [documentation](https://docs.r2_robotic_arm.dev/software/can/)
- 💬 Join the community on [Discord](https://discord.gg/FsZaZ4z3We)
- 📬 Contact us through <r2_robotic_arm@enactic.ai>

## License

Licensed under the Apache License 2.0. See `LICENSE.txt` for details.

Copyright 2025 Enactic, Inc.

## Code of Conduct

All participation in the R2RoboticArm project is governed by our [Code of Conduct](CODE_OF_CONDUCT.md).
