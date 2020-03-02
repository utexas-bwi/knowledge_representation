DROP SCHEMA public CASCADE;
CREATE SCHEMA public;
GRANT ALL ON SCHEMA public TO postgres;
GRANT ALL ON SCHEMA public TO public;

CREATE TABLE entities
(
    entity_id SERIAL NOT NULL,
    PRIMARY KEY (entity_id)
);

CREATE TYPE attribute_type as ENUM ('bool', 'int', 'id', 'str', 'float');

CREATE TABLE attributes
(
    attribute_name varchar(24)    NOT NULL,
    type           attribute_type NOT NULL,
    PRIMARY KEY (attribute_name)
);

CREATE TABLE concepts
(
    entity_id int NOT NULL,
    concept_name varchar(24) NOT NULL UNIQUE,
    PRIMARY KEY (entity_id, concept_name),
        FOREIGN KEY (entity_id)
        REFERENCES entities (entity_id)
        ON DELETE CASCADE
        ON UPDATE CASCADE
);

CREATE TABLE instance_of
(
    entity_id int NOT NULL,
    concept_name varchar(24) NOT NULL,
    PRIMARY KEY (entity_id, concept_name),
    FOREIGN KEY (entity_id)
        REFERENCES entities (entity_id)
        ON DELETE CASCADE
        ON UPDATE CASCADE,
    FOREIGN KEY (concept_name)
        REFERENCES concepts (concept_name)
        ON DELETE CASCADE
        ON UPDATE CASCADE
);

/******************* ENTITY ATTRIBUTES */

CREATE TABLE entity_attributes_id
(
    entity_id       int         NOT NULL,
    attribute_name  varchar(24) NOT NULL,
    attribute_value int         NOT NULL,
    PRIMARY KEY (entity_id, attribute_name, attribute_value),
    FOREIGN KEY (attribute_value)
        REFERENCES entities (entity_id)
        ON DELETE CASCADE
        ON UPDATE CASCADE,
    FOREIGN KEY (entity_id)
        REFERENCES entities (entity_id)
        ON DELETE CASCADE
        ON UPDATE CASCADE,
    FOREIGN KEY (attribute_name)
        REFERENCES attributes (attribute_name)
        ON DELETE CASCADE
        ON UPDATE CASCADE
);

CREATE TABLE entity_attributes_str
(
    entity_id       int         NOT NULL,
    attribute_name  varchar(24) NOT NULL,
    attribute_value varchar(24) NOT NULL,
    PRIMARY KEY (entity_id, attribute_name, attribute_value),
    FOREIGN KEY (entity_id)
        REFERENCES entities (entity_id)
        ON DELETE CASCADE
        ON UPDATE CASCADE,
    FOREIGN KEY (attribute_name)
        REFERENCES attributes (attribute_name)
        ON DELETE CASCADE
        ON UPDATE CASCADE/*,
    CONSTRAINT CHK_name_unique CHECK (NumSameNames() = 0)*/
);

CREATE TABLE entity_attributes_float
(
    entity_id       int         NOT NULL,
    attribute_name  varchar(24) NOT NULL,
    attribute_value float       NOT NULL,
    PRIMARY KEY (entity_id, attribute_name, attribute_value),
    FOREIGN KEY (entity_id)
        REFERENCES entities (entity_id)
        ON DELETE CASCADE
        ON UPDATE CASCADE,
    FOREIGN KEY (attribute_name)
        REFERENCES attributes (attribute_name)
        ON DELETE CASCADE
        ON UPDATE CASCADE
);

CREATE TABLE entity_attributes_bool
(
    entity_id       int         NOT NULL,
    attribute_name  varchar(24) NOT NULL,
    attribute_value bool,
    PRIMARY KEY (entity_id, attribute_name, attribute_value),
    FOREIGN KEY (entity_id)
        REFERENCES entities (entity_id)
        ON DELETE CASCADE
        ON UPDATE CASCADE,
    FOREIGN KEY (attribute_name)
        REFERENCES attributes (attribute_name)
        ON DELETE CASCADE
        ON UPDATE CASCADE
);

/******************* MAPS */

CREATE TABLE points
(
    entity_id int NOT NULL,
    point point NOT NULL,
    PRIMARY KEY (entity_id),
    FOREIGN KEY (entity_id)
        REFERENCES entities (entity_id)
        ON DELETE CASCADE
        ON UPDATE CASCADE
);

CREATE TABLE regions
(
    entity_id int NOT NULL,
    region polygon NOT NULL,
    PRIMARY KEY (entity_id),
    FOREIGN KEY (entity_id)
        REFERENCES entities (entity_id)
        ON DELETE CASCADE
        ON UPDATE CASCADE
);

/*TODO(nickswalker): These can have magnitude normalization*/
CREATE TABLE poses
(
    entity_id int NOT NULL,
    pose lseg NOT NULL,
    PRIMARY KEY (entity_id),
    FOREIGN KEY (entity_id)
        REFERENCES entities (entity_id)
        ON DELETE CASCADE
        ON UPDATE CASCADE
);

CREATE TABLE doors
(
    entity_id int NOT NULL,
    door lseg NOT NULL,
    PRIMARY KEY (entity_id),
    FOREIGN KEY (entity_id)
        REFERENCES entities (entity_id)
        ON DELETE CASCADE
        ON UPDATE CASCADE
);

/******************* FUNCTIONS */

/* Counts the number of entities that have the same name */
CREATE FUNCTION NumSameNames()
    RETURNS BIGINT
    IMMUTABLE
    LANGUAGE SQL
AS
$$
/* TODO: Fix this and reenable the trigger */
SELECT COUNT(*)
FROM entity_attributes_str
WHERE attribute_name = 'name'
  AND attribute_value = 'robot'
$$;

CREATE FUNCTION remove_attribute(INT, varchar(24))
    RETURNS BIGINT
    LANGUAGE plpgsql
AS
$body$
DECLARE
    n_bool_del  bigint;
    n_float_del bigint;
    n_id_del    bigint;
    n_str_del   bigint;
BEGIN
    WITH bool_del
             AS (DELETE FROM entity_attributes_bool WHERE entity_id = $1 AND attribute_name = $2 RETURNING entity_id)
    SELECT count(*)
    FROM bool_del
    INTO n_bool_del;
    WITH float_del
             AS (DELETE FROM entity_attributes_float WHERE entity_id = $1 AND attribute_name = $2 RETURNING entity_id)
    SELECT count(*)
    FROM float_del
    INTO n_float_del;
    WITH id_del AS (DELETE FROM entity_attributes_id WHERE entity_id = $1 AND attribute_name = $2 RETURNING entity_id)
    SELECT count(*)
    FROM id_del
    INTO n_id_del;
    WITH str_del AS (DELETE FROM entity_attributes_str WHERE entity_id = $1 AND attribute_name = $2 RETURNING entity_id)
    SELECT count(*)
    FROM str_del
    INTO n_str_del;
    RETURN n_bool_del + n_float_del + n_id_del + n_str_del;
END
$body$;

CREATE FUNCTION get_concepts_recursive(INT)
    RETURNS TABLE
            (
                entity_id INT,
                concept_name varchar(24)
            )
    IMMUTABLE
    LANGUAGE SQL
AS
$$
WITH RECURSIVE cteConcepts (ID)
   AS
   (
       /* Get whatever the argument is an instance of, and then every thing that that is a descended concept of*/
       SELECT concepts.entity_id
       FROM concepts
       INNER JOIN instance_of ON (instance_of.entity_id = $1 AND instance_of.concept_name = concepts.concept_name)

       UNION ALL

       SELECT a.attribute_value
       FROM entity_attributes_id a
                INNER JOIN cteConcepts b
                           ON a.attribute_name = 'is_a'
                               AND a.entity_id = b.ID
   )
SELECT entity_id, concept_name
FROM cteConcepts INNER JOIN concepts ON entity_id = ID;
$$;

CREATE FUNCTION get_all_concept_ancestors(INT)
    RETURNS TABLE
            (
                entity_id INT,
                concept_name varchar(24)
            )
    IMMUTABLE
    LANGUAGE SQL
AS
$$
WITH RECURSIVE cteConcepts (id)
   AS
   (
       /* Make sure the argument is a concept */
       SELECT entity_id FROM concepts WHERE (entity_id = $1)
       UNION ALL
       SELECT a.attribute_value
       FROM entity_attributes_id a
                INNER JOIN cteConcepts b
                           ON a.attribute_name = 'is_a'
                               AND a.entity_id = b.ID
   )
SELECT entity_id, concept_name
FROM cteConcepts INNER JOIN concepts ON entity_id = ID;
$$;


CREATE FUNCTION get_all_concept_descendants(INT)
    RETURNS TABLE
            (
                entity_id INT,
                concept_name varchar(24)
            )
    IMMUTABLE
    LANGUAGE SQL
AS
$$
WITH RECURSIVE cteConcepts (id)
AS
(
   /* Make sure the argument is a concept */
   SELECT entity_id FROM concepts WHERE (entity_id = $1)
   UNION ALL
   SELECT a.entity_id
   FROM entity_attributes_id a
            INNER JOIN cteConcepts b
                       ON a.attribute_name = 'is_a'
                        AND a.attribute_value = b.ID
)
SELECT entity_id, concept_name
FROM cteConcepts INNER JOIN concepts ON entity_id = ID;
$$;


CREATE FUNCTION get_all_instances_of_concept_recursive(INT)
    RETURNS TABLE
            (
                entity_id INT,
                concept_name varchar(24)
            )
    IMMUTABLE
    LANGUAGE SQL
AS
$$
WITH RECURSIVE cteConcepts (id)
AS
(
   /* Make sure the argument is a concept */
   SELECT entity_id FROM concepts WHERE (entity_id = $1)
   UNION ALL
   SELECT a.entity_id
   FROM entity_attributes_id a
            INNER JOIN cteConcepts b
                       ON a.attribute_name = 'is_a'
                        AND a.attribute_value = b.ID
)
SELECT entity_id, concept_name
FROM instance_of WHERE concept_name IN (SELECT concept_name FROM cteConcepts INNER JOIN concepts ON (concepts.entity_id = id));
$$;

CREATE FUNCTION add_default_attributes()
    RETURNS VOID
    LANGUAGE SQL
AS
$$

INSERT INTO attributes
VALUES ('answer_to', 'int');
INSERT INTO attributes
VALUES ('default_location', 'int');
INSERT INTO attributes
VALUES ('has', 'id');
INSERT INTO attributes
VALUES ('height', 'float');
INSERT INTO attributes
VALUES ('is_a', 'int');
INSERT INTO attributes
VALUES ('is_connected', 'id');
INSERT INTO attributes
VALUES ('is_delivered', 'id');
INSERT INTO attributes
VALUES ('is_facing', 'id');
INSERT INTO attributes
VALUES ('is_holding', 'id');
INSERT INTO attributes
VALUES ('is_in', 'id');
INSERT INTO attributes
VALUES ('is_near', 'id');
INSERT INTO attributes
VALUES ('is_open', 'bool');
INSERT INTO attributes
VALUES ('is_placed', 'id');
INSERT INTO attributes
VALUES ('name', 'str');
INSERT INTO attributes
VALUES ('part_of', 'id')
$$;

/***** DEFAULT VALUES */
CREATE FUNCTION add_default_entities()
    RETURNS bigint
    LANGUAGE SQL
AS
$$

INSERT INTO entities
VALUES (1);
INSERT INTO entities
VALUES (2);
INSERT INTO concepts
VALUES (2, 'robot');
INSERT INTO instance_of
VALUES (1, 'robot');

/* Manual inserts will mess up the SERIAL sequence, so we have to manually bump the number*/
SELECT setval('entities_entity_id_seq', max(entity_id))
FROM   entities;
$$;

SELECT *
FROM add_default_attributes();
SELECT *
FROM add_default_entities();
