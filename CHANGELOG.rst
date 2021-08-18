^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package knowledge_representation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.9.5 (2021-08-17)
------------------
* Modify build configuration to support Debian Buster
* Contributors: Nick Walker

0.9.4 (2021-08-16)
------------------
* Add lock capability to LTMC
* Improve support for SVG transforms in map annotations
* Document map annotation process
* Improve example annotated map
* Contributors: Nick Walker, Yuqian Jiang, Tobias Fischer

0.9.3 (2021-02-06)
------------------
* Add door type
* Add script for making a PGM that has annotated doors as obstacles
* Update annotation loader to look for groups that look like door annotations
* Fix "contained" queries and use in show_me
* Minor documentation improvements
* Support "instance_of" in knowledge loader
* Fix API for SELECT queries on entity_attributes tables
* Contributors: Nick Walker, Yuqian Jiang

0.9.2 (2020-07-26)
------------------
* Fix errant Python2 style print
* Build Python3 bindings for new distributions
* Conditional dependencies for Python2/Python3 image library
* Use setuptools always
* Bump CMake version to avoid CMP0048
* Contributors: Nick Walker, Shane Loretz

0.9.1 (2020-07-23)
-------------------
* Update YAML knowledge_loader
* Support passing multiple maps to the map loader
* Update rosdeps for Noetic

0.9.0 (2020-07-07)
------------------
* Initial release
