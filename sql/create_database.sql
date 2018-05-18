drop database if exists villa_krr;
create database villa_krr;
use villa_krr;

CREATE TABLE objects (
    object_id int NOT NULL AUTO_INCREMENT,
    PRIMARY KEY(object_id)
);

CREATE TABLE attributes (
    attribute_name varchar(24) NOT NULL,
    PRIMARY KEY(attribute_name)
);

CREATE TABLE object_attributes (
    object_id int NOT NULL,
    attribute_name varchar(24) NOT NULL,
    attribute_value_object_id int,
    attribute_value_string varchar(24),
    attribute_value_float float,
    attribute_value_bool bool,
    PRIMARY KEY(object_id, attribute_name),
    FOREIGN KEY(attribute_value_object_id)
        REFERENCES objects(object_id)
        ON DELETE CASCADE
        ON UPDATE CASCADE,
    FOREIGN KEY(object_id)
        REFERENCES objects(object_id)
        ON DELETE CASCADE
        ON UPDATE CASCADE,
    FOREIGN KEY(attribute_name)
        REFERENCES attributes(attribute_name)
        ON DELETE CASCADE
        ON UPDATE CASCADE
);

insert into attributes(attribute_name) values('concept');
insert into attributes(attribute_name) values('is_a');
insert into attributes(attribute_name) values('is_on');
insert into attributes(attribute_name) values('is_in');
insert into attributes(attribute_name) values('facing');
insert into attributes(attribute_name) values('located');
insert into attributes(attribute_name) values('sensed');
insert into attributes(attribute_name) values('person_name');
insert into attributes(attribute_name) values('location');
insert into attributes(attribute_name) values('found_in');
insert into attributes(attribute_name) values('question');
insert into attributes(attribute_name) values('answer_to');

/***** DEFAULT VALUES */

insert into objects(object_id) values(1);
insert into objects(object_id) values(2);
insert into object_attributes(object_id, attribute_name, attribute_value_string) values(2, 'concept', 'robot');
insert into object_attributes(object_id, attribute_name, attribute_value_string) values(1, 'is_a', 'robot');


/*
How many chairs are in the livingroom?

First - Select everything that's in something.
select * from object_attributes where attribute_name='is_in'

Second - Select the concept of a livingroom.
select * from object_attributes where attribute_name='concept' and attribute_value_string='livingroom'

Third - Select the objects that are livingrooms.
select object_id from (select t1.object_id from object_attributes t1, object_attributes t2 where t1.attribute_value_object_id = t2.object_id and t1.attribute_name='is_a' and t2.attribute_name='concept' and t2.attribute_value_string='livingroom') t3;

Third - Select everything that's a chair.
select object_id from (select t1.object_id from object_attributes t1, object_attributes t2 where t1.attribute_value_object_id = t2.object_id and t1.attribute_name='is_a' and t2.attribute_name='concept' and t2.attribute_value_string='chair') t4;

Fourth - Select every chair's is_in attribute;
select * from object_attributes t5 inner join t5.object_id in (select object_id from (select t1.object_id from object_attributes t1, object_attributes t2 where t1.attribute_value_object_id = t2.object_id and t1.attribute_name='is_a' and t2.attribute_name='concept' and t2.attribute_value_string='chair') t4);
*/

/*
Where is the coke?

select attribute_value_string from object_attributes where attribute_name='concept' and object_id=(select attribute_value_object_id from object_attributes where attribute_name='is_a' and object_id=(select attribute_value_object_id from object_attributes where attribute_name='is_on' and object_id=(select object_id from object_attributes where attribute_name='is_a' and attribute_value_object_id=(select object_id from object_attributes where attribute_value_string='coke'))));

This gets you the type of the thing that the coke is on.
select attribute_value_object_id from object_attributes where attribute_name='is_a' and object_id=(select attribute_value_object_id from object_attributes where attribute_name='is_on' and object_id=(select object_id from object_attributes where attribute_name='is_a' and attribute_value_object_id=(select object_id from object_attributes where attribute_value_string='coke')));

This gets you the thing that the coke is on.
select attribute_value_object_id from object_attributes where attribute_name='is_on' and object_id=(select object_id from object_attributes where attribute_name='is_a' and attribute_value_object_id=(select object_id from object_attributes where attribute_value_string='coke'));

This gets you the concrete 'coke' object.
select object_id from object_attributes where attribute_name='is_a' and attribute_value_object_id=(select object_id from object_attributes where attribute_value_string='coke');

insert into objects(object_id) values(3);
insert into object_attributes(object_id, attribute_name, attribute_value_object_id) values(3, 'is_a', 0);
*/
