# 1. Use cmake 3.15
# 2. build gtsam, install python and wrap\
# 3. use gtsam-project-python as a test


cmake .. -DPYTHON_EXECUTABLE:FILEPATH=/home/jerry/Projects/Motion/env/bin/python \
-DGPMP2_BUILD_PYTHON=1 \
-DPython_EXECUTABLE=/home/jerry/Projects/Motion/env/bin/python