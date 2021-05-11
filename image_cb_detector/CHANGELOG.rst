^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package image_cb_detector
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.10.15 (2021-05-11)
--------------------
* Merge pull request `#47 <https://github.com/ros-perception/calibration/issues/47>`_ from PR2-prime/noetic-devel
  Noetic devel
* fixes for compiler warnings CMP0048 and CMP0046
* update maintainer for most packages; first try at setup for travis testing (`#46 <https://github.com/ros-perception/calibration/issues/46>`_)
* fixed unit tests for compile/testing in focal/noetic/opencv4
* Contributors: Dave Feil-Seifer

0.10.14 (2016-04-16)
--------------------
* simplify OpenCV3 dependency
* Contributors: Vincent Rabaud

0.10.13 (2014-12-21)
--------------------

0.10.12 (2014-09-21)
--------------------
* make code compile with OpenCV3
* Contributors: Vincent Rabaud

0.10.11 (2014-09-20)
--------------------
* check if input depth image is 32bit
  also fixes `#29 <https://github.com/ros-perception/calibration/issues/29>`_ with proper log message
* Contributors: Vincent Rabaud

0.10.9 (2014-04-09)
-------------------
* remove PCL dependency
* Contributors: Vincent Rabaud

0.10.8 (2014-03-07)
-------------------
* no need to depend on opencv2 as we depend on cv_bridge
  This is useful to only have one branch for both hydro and indigo
* Contributors: Vincent Rabaud
