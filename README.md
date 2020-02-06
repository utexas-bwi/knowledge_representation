# knowledge_representation [![CI](https://github.com/utexas-bwi/knowledge_representation/workflows/CI/badge.svg)](https://github.com/utexas-bwi/knowledge_representation/actions?query=workflow%3ACI)

Mechanisms for storing and querying information about the world.

* Persistent storage backed by a local PostgreSQL or MySQL database
* Convenient API's for common queries, like getting all instances of a class of object
* Link directly against your C++ code, or import a Python binding

## Installation

Make sure you've pulled down the package dependencies using rosdep

    rosdep install --from-paths . -ry

Then run one of `scripts/configure_{mysql,postgresql}.sh` to install the default database configuration and schema. **PostgreSQL is the preferred backend.**

## Usage

For example usage of the C++ API, check out `test/ltmc.cpp`. For Python, scripts `test_ltmc`.

## Development

If you're working on the MySQL interface, we access the backing store via the xdev API. See the [documentation](https://dev.mysql.com/doc/dev/connector-cpp/8.0/) for the official MySQL xdev API C++ library.