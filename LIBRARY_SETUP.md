# Library Setup Guide

## What You Need

## Figure out a way to integrate the IMU into the code and send data through UART.

Your instructor provides two pre-built libraries for the TM4C123 target:

| File | Purpose |
|------|---------|
| `libcybotUART.lib` | Pre-built UART library for the CyBot |
| `libcybotScan.lib` | Pre-built scanning library for the CyBot |

Get these files from your instructor or course portal. They are `.lib` files built for the TM4C123GH6PM target.

---

## Which Labs Need Which Libraries

| Lab | libcybotUART.lib | libcybotScan.lib |
|-----|:-:|:-:|
| Lab2 | Yes | No |
| Lab3 | Yes | Yes |
| Lab4 | Yes | No |
| Lab5 | Yes | No |
| Lab6 | Yes | No |
| Lab7 | Yes | Yes |

---

## How to Add a Library to a Project in CCS

### Step 1 — Copy the .lib file into the lab folder

Place the `.lib` file directly inside the lab folder (e.g., `Lab2/libcybotUART.lib`).

### Step 2 — Add the library file to the project

1. In the **Project Explorer**, right-click the lab project (e.g., `CPRE288_Lab2`)
2. Select **Add Files...**
3. Navigate to the lab folder and select the `.lib` file
4. Click **OK** — choose **Copy files** if prompted

### Step 3 — Add the library to the linker file search path

1. Right-click the project → **Properties**
2. Go to **Build → ARM Linker → File Search Path**
3. Under **Include library file or command file as input (--library)**, click the **Add** button (green `+`)
4. Type the filename exactly: `libcybotUART.lib` (or `libcybotScan.lib`)
5. Click **OK** → **Apply and Close**

### Step 4 — Verify the headers are present

The headers `cyBot_uart.h` and `cyBot_Scan.h` are already copied into each lab folder, so `#include "cyBot_uart.h"` and `#include "cyBot_Scan.h"` should resolve without any extra steps.

---

## Also Required: TivaWare Include Path

All projects need the TivaWare SDK include path set before they will compile.

1. Right-click the project → **Properties**
2. Go to **Build → ARM Compiler → Include Options**
3. Under **Add dir to #include search path**, click **Add** (green `+`)
4. Enter the path to your TivaWare installation:
   - Windows: `C:\ti\TivaWare_C_Series-2.2.0.295`
   - Adjust the version number to match what you have installed
5. Click **OK** → **Apply and Close**

Keep this path consistent on the Windows computer when possible. This repository assumes:

```text
C:\ti\TivaWare_C_Series-2.2.0.295
```

If TivaWare is installed somewhere else, update the path in CCS after import instead of committing machine-specific workspace metadata.

---

## Importing Projects in CCS

For the cleanest import on Windows with Code Composer Studio 12.7.1:

1. Pull the latest repository changes from GitHub, or download the repository zip and extract it.
2. Open Code Composer Studio with a fresh workspace directory outside this repository.
3. Choose **File → Import → Code Composer Studio → CCS Projects**.
4. Select the specific lab folder, such as `Lab7`, `IMU`, or `LABs_f_d`, rather than selecting the whole repository as one project.
5. Leave generated workspace metadata out of Git.

If you downloaded a zip from GitHub, the extracted folder may be named like `CPRE-2880-SA1-ping-ir`. Point CCS at a project folder inside it:

```text
CPRE-2880-SA1-ping-ir\Lab7
CPRE-2880-SA1-ping-ir\IMU
CPRE-2880-SA1-ping-ir\LABs_f_d
```

Avoid importing the outer extracted folder as a CCS project.

The files `.project`, `.cproject`, `.ccsproject`, `targetConfigs/`, source files, headers, `.cmd` linker files, and required `.lib` files are the project files that should travel through Git.

---

## Also Required: Linker Command File

Each project needs a linker command file (`tm4c123gh6pm.cmd`) that tells the linker how to lay out code in the TM4C123's Flash and SRAM.

**Option A — Use the TivaWare copy (recommended):**

1. Right-click the project → **Properties**
2. Go to **Build → ARM Linker → Advanced Options → Command Files**
3. Click **Add** (green `+`)
4. Enter the path to TivaWare's linker file:
   `C:\ti\TivaWare_C_Series-2.2.0.295\examples\boards\ek-tm4c123gxl\blinky\blinky.ld`
   — or search your TivaWare folder for `tm4c123gh6pm.cmd`
5. Click **OK** → **Apply and Close**

**Option B — Get the file from your instructor and add it manually:**

1. Copy `tm4c123gh6pm.cmd` into the lab folder
2. Right-click the project → **Add Files...** and select it
3. CCS will automatically detect and use `.cmd` files added to the project

---

## Also Required: Startup File

CCS needs a startup file (`tm4c123gh6pm_startup_ccs.c`) with the interrupt vector table.

1. Create a new empty CCS project for the TM4C123GH6PM target
2. Copy the auto-generated `tm4c123gh6pm_startup_ccs.c` from that project
3. Add it to each lab project via **Add Files...**
