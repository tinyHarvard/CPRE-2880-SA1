# CPRE-2880-SA1
CprE 2880 CyBot code for team SA1-1

## Editing on macOS and building on Windows

Use this repository as the source of truth. Edit source files in VS Code on macOS, commit and push those changes to GitHub, then pull the same branch on the Windows computer that has Code Composer Studio installed.

Recommended workflow:

1. On macOS, edit the lab/source files in VS Code.
2. Check your local changes:
   ```sh
   git status
   ```
3. Commit only the files you intentionally changed:
   ```sh
   git add <file-or-folder>
   git commit -m "Describe the change"
   git push origin <branch-name>
   ```
4. On Windows, clone once or pull updates:
   ```sh
   git clone https://github.com/tinyHarvard/CPRE-2880-SA1.git
   ```
   or, inside an existing clone:
   ```sh
   git pull
   ```
   If you download the repository as a GitHub zip instead, extract the zip first. GitHub usually creates a folder named like `CPRE-2880-SA1-ping-ir`.
5. In Code Composer Studio 12.7.1, use a fresh CCS workspace, then import the lab project folder you want to build.

When using a downloaded zip, import a specific extracted project folder such as:

```text
CPRE-2880-SA1-ping-ir\Lab7
CPRE-2880-SA1-ping-ir\IMU
CPRE-2880-SA1-ping-ir\LABs_f_d
```

Do not import the outer `CPRE-2880-SA1-ping-ir` folder as one CCS project.

Do not commit CCS workspace folders such as `.metadata/`, `.jxbrowser.userdata/`, `.settings/`, `.launches/`, `Debug/`, or `Release/`. Those are local generated files and often cause import problems on another computer.

For CCS setup details, see [LIBRARY_SETUP.md](LIBRARY_SETUP.md).
