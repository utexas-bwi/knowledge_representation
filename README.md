# knowledge_representation [![CI](https://github.com/utexas-bwi/knowledge_representation/workflows/CI/badge.svg)](https://github.com/utexas-bwi/knowledge_representation/actions?query=workflow%3ACI)

Mechanisms for storing and querying information about the world. You can think of this package as a nicely packaged [database schema](https://en.wikipedia.org/wiki/Database_schema); it's an opinionated take on how to structure the kinds of knowledge that a robot might need and tools for [CRUD operations](https://en.wikipedia.org/wiki/Create,_read,_update_and_delete) within that structure.

* Persistent storage backed by a local PostgreSQL or MySQL database
* Convenient API's for common queries, like getting all instances of a class of object
* Link directly against your C++ code, or import a Python binding

## Representation

Knowledge is broken up into _concepts_, _instances_ and _relations_.

**Concepts** are abstract ideas. They represent the _idea_ of something. For example, the general idea of an apple but not any one apple in particular.

**Instances** are concrete, usually physical manifestations of concepts. You could have a dozen instances of a concept like apple.

Both concepts and instances are **entities** in the knowledgebase. Each entity is granted a unique identifying number.

**Relations** tie different entities together and store data about them. Each relation has a name and a type. For example, an apple instance might be `on` some other instance. The relation is named `on` and its type is `entity_id`, because it relates some entity to another. Relations with types other than `entity_id` function more like properties; for example, the relation `is_graspable` with a type of `bool` is just storing a simple fact about an entity.

### Map Types

Mobile robots often have to answer questions like "what room am I in?" or "which is the nearest X?". knowledge_representation includes special affordances for representing 2D geometric information that can be queried to answer these kinds of questions:

* **Points** are a special kind of instance which store an x and y coordinate.
* **Poses** are a special kind of instance which store an x and y coordinate as well as a direction.
* **Regions** are a special kind of instance which store a list of x and y coordinates defining a closed region.

All of these types are uniquely tied to a single **map**, a special kind of instance.

Like concepts, all of these geometric types _must_ have names configured. Without names attached, there isn't enough semantic information to support any kind of useful operation. What would it mean for a point to be in a region if neither the point nor the region had names?

## Installation

Make sure you've pulled down the package dependencies using rosdep

    rosdep install --from-paths . -ry

Then run one of `scripts/configure_{mysql,postgresql}.sh` to install the default database configuration and schema. **PostgreSQL is the preferred backend.**

## Usage

For example usage of the C++ API, check out `test/ltmc.cpp`. For Python, scripts `test_ltmc`.

## Development

If you're working on the MySQL interface, we access the backing store via the xdev API. See the [documentation](https://dev.mysql.com/doc/dev/connector-cpp/8.0/) for the official MySQL xdev API C++ library.