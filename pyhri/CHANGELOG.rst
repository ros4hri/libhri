^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pyhri
^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.3.0 (2024-03-18)
------------------
* change python module name to hri
* Contributors: Luka Juricic

2.2.0 (2024-01-18)
------------------

2.1.0 (2024-01-18)
------------------

Major new release, moved from ROS 1 to ROS 2, based on libhri + pybind11.

While the overall pyhri API should not have changed in significant ways, this
release *only works with ROS 2*!

Detailed changelog:

* convert IntensityConfidence and PointOfInterest to tuple
* add documentation
* fix version, documentation, example install
* add pyhri example
* move rivate headers to src
* cleanup CMakeLists.txt
* add test dependencies
* add tf listener to the callback group
* fix external code acknowledgement
* minor adaptations to ndarray_converter
* add external code ndarray_converter for image cast
* add pyhri tests and align hri ones with the former
* add pybind11 based python bindings to pyhri
* add pyhri python bindings package
* Contributors: Luka Juricic

0.4.1 (2023-07-05)
------------------
* fix voice callbacks test
* change RoI message type to hri_msgs/NormalizedRegionOfInterest2D
* Contributors: Luka Juricic

0.4.0 (2023-04-12)
------------------
* {hri->pyhri}
  This was causing catkin to not find pyhri when included in other projects
* add callbacks for speech recognition + tests
* fix AttributeError on first detection
* use sensor_msgs/RegionOfInterest msg for compatibility with other ROS4HRI repos
* Contributors: Luca Pozzi, Séverin Lemaignan

0.3.2 (2022-10-25)
------------------
* fix RoI-related message types. Unit-tests pass again.
* Contributors: Séverin Lemaignan

0.3.1 (2022-10-26)
------------------
* voice: add support for is_speaking
* ensure known persons get their face/body/voice instances properly updated
  Fixes #4
* Contributors: Séverin Lemaignan

0.3.0 (2022-10-22)
------------------
* wire up callbacks for when features appear/disappear
* body: currently, hri_fullbody (incorrectly) returns a RegionOfInterest for the RoI, instead of a NormalisedRegionOfInterest2D
* body: skeleton2d now gives direct access to the list of joints coordinates
* voice: expose last recognised speech + transform
* use @property to make the API simpler and safer
  In particular, to transparently provide *copies* of the faces, bodies, voices, persons list in HRIListener
* body: expose skeleton2d, transform, {RegionOfInterest -> NormalizedRegionOfInterest2D} + doc
* face: {RegionOfInterest -> NormalizedRegionOfInterest2D} + doc
* Contributors: Séverin Lemaignan

0.2.0 (2022-10-18)
------------------
* autogenerate docs with rosdoc (for display on ROS wiki) and SPHINX
* More api documentation
* adjust code base for ReadTheDocs doc generation
  In particular:
  - add a requirements.txt file for catkin
  - allows importing pyhri without rospy
* Face.transform|Face.gaze_transform can change their reference frame
* removed dep on numpy.typing as it is not widely available yet
* mark features as invalid once they disappear
* returns copy of persons to avoid modifying dictionary while iterating
  While here, add infrastrcutre to check whether a face/body/voice/person is still valid
* expose the tf transform of the face and gaze
* add BSD license file
* [test] further tuning of tests' waiting behaviours for #1
* Contributors: Séverin Lemaignan

0.1.3 (2022-07-13)
------------------
* refining waiting + closing in tests -- they all pass reliably on my machine
* unregister from main topics when closing HRIListener
* Contributors: Séverin Lemaignan

0.1.2 (2022-07-12)
------------------
* [cmake] add test only behing CATKIN_ENABLE_TESTING
* Contributors: Séverin Lemaignan

0.1.1 (2022-07-06)
------------------
* various fixes after first test with actual ROS4HRI messages
* Contributors: Séverin Lemaignan

0.1.0 (2022-07-06)
------------------
* initial implementation, closely following the libhri C++ API
* all unit-tests pass (modulo random timing/waiting issues)
* add basic README
* Contributors: Séverin Lemaignan
