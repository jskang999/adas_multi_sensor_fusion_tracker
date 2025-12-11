# Third-Party Notices

This project depends on the following third-party components. The project
itself does not bundle their source code; they are expected to be installed
via system packages or pip.

---

## Eigen3

- Description: C++ template library for linear algebra
- Usage: Used through the system package `libeigen3-dev` as a header-only
  dependency for matrix/vector operations in the tracking code.
- License: MPL-2.0 (Mozilla Public License 2.0)
- Official information:
  - https://eigen.tuxfamily.org/
  - https://www.mozilla.org/MPL/2.0/

---

## Matplotlib

- Description: Python plotting library used for visualization scripts
  (`tools/plot_tracks.py`, `tools/visualize_image_with_tracks.py`).
- Installation: Installed via `pip install matplotlib`.
- License: Matplotlib license (BSD-style, PSF-based)
- Official information:
  - https://matplotlib.org/stable/project/license.html
  - https://pypi.org/project/matplotlib/