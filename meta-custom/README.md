# meta-custom: BSP Layer for IMU

This Yocto/OpenEmbedded layer provides custom kernel recipes and drivers required for the **IMU** project, specifically focusing on the i.MX8M Plus platform and the ICM20948 IMU sensor.

---

## 🛠️ Layer Contents

### 1. Recipes-Kernel
*   **`icm20948`**: A custom out-of-tree kernel module for the ICM20948 9-axis sensor (Accelerometer, Gyroscope, Magnetometer).
    *   Supports IOCTL interface for high-frequency data sampling.
    *   Compatible with I2C/SPI interfaces (configured for I2C by default in this recipe).

---

## 🏗️ Dependencies

This layer depends on:

*   **URI**: `git://git.openembedded.org/openembedded-core`
*   **Layers**: `meta`
*   **Branch**: `scarthgap` (or compatible)

---

## 🚀 Installation & Usage

### 1. Adding the layer to your build
To include this layer in your Yocto environment, run the following command from your build directory:

```bash
bitbake-layers add-layer ../sources/meta-custom
```

### 2. Including the driver in your image
Add the following to your `conf/local.conf` or your custom image recipe:

```bitbake
IMAGE_INSTALL:append = " icm20948"
KERNEL_MODULE_AUTOLOAD += "icm20948"
```

---

## 🔧 Maintenance

### Maintainer
*   **Name**: Vishnu Siddavatam (Smart IP Team)
*   **Email**: vishnuvardhan.siddavatam@vconnectech.in

### License
The recipes in this layer are licensed under the **MIT License** (see [COPYING.MIT](COPYING.MIT)), and the kernel modules are licensed under **GPLv2**.

