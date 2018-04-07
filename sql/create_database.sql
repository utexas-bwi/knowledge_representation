drop database if exists villa_krr;
create database villa_krr;
use villa_krr;

create table objects (
    object_id int NOT NULL AUTO_INCREMENT,
    primary key(object_id)
);

create table attributes (
    attribute_name varchar(24) NOT NULL,
    primary key(attribute_name)
);

create table object_attributes (
    object_id int NOT NULL,
    attribute_name varchar(24) NOT NULL,
    attribute_value_object_id int,
    attribute_value_string varchar(24),
    attribute_value_float float,
    attribute_value_bool boolean,
    primary key(object_id, attribute_name),
    foreign key(attribute_value_object_id) references objects(object_id)
);

insert into attributes(attribute_name) values('concept');
insert into attributes(attribute_name) values('is_a');
insert into attributes(attribute_name) values('is_on');
insert into attributes(attribute_name) values('is_in');

insert into objects(object_id) values(0);
insert into object_attributes(object_id, attribute_name, attribute_value_string) values(0, 'concept', 'coke');

insert into objects(object_id) values(1);
insert into object_attributes(object_id, attribute_name, attribute_value_string) values(1, 'concept', 'surface');

insert into objects(object_id) values(2);
insert into object_attributes(object_id, attribute_name, attribute_value_string) values(2, 'concept', 'table');

insert into object_attributes(object_id, attribute_name, attribute_value_object_id) values(2, 'is_a', 1);

insert into objects(object_id) values(3);
insert into object_attributes(object_id, attribute_name, attribute_value_object_id) values(3, 'is_a', 2);

insert into objects(object_id) values(4);
insert into object_attributes(object_id, attribute_name, attribute_value_object_id) values(4, 'is_a', 0);
insert into object_attributes(object_id, attribute_name, attribute_value_object_id) values(4, 'is_on', 3);



insert into objects(object_id) values(10);
insert into objects(object_id) values(11);
insert into objects(object_id) values(12);

insert into object_attributes(object_id, attribute_name, attribute_value_string) values(10, 'concept', 'chair');

insert into object_attributes(object_id, attribute_name, attribute_value_string) values(11, 'concept', 'room');
insert into object_attributes(object_id, attribute_name, attribute_value_string) values(12, 'concept', 'livingroom');
insert into object_attributes(object_id, attribute_name, attribute_value_object_id) values(12, 'is_a', 11);

insert into objects(object_id) values(100);
insert into object_attributes(object_id, attribute_name, attribute_value_object_id) values(100, 'is_a', 12);

insert into objects(object_id) values(13);
insert into objects(object_id) values(14);
insert into objects(object_id) values(15);
insert into objects(object_id) values(16);
insert into objects(object_id) values(17);
insert into object_attributes(object_id, attribute_name, attribute_value_object_id) values(13, 'is_a', 10);
insert into object_attributes(object_id, attribute_name, attribute_value_object_id) values(14, 'is_a', 10);
insert into object_attributes(object_id, attribute_name, attribute_value_object_id) values(15, 'is_a', 10);
insert into object_attributes(object_id, attribute_name, attribute_value_object_id) values(16, 'is_a', 10);
insert into object_attributes(object_id, attribute_name, attribute_value_object_id) values(17, 'is_a', 10);

insert into object_attributes(object_id, attribute_name, attribute_value_object_id) values(13, 'is_in', 100);
insert into object_attributes(object_id, attribute_name, attribute_value_object_id) values(14, 'is_in', 100);
insert into object_attributes(object_id, attribute_name, attribute_value_object_id) values(15, 'is_in', 100);
insert into object_attributes(object_id, attribute_name, attribute_value_object_id) values(16, 'is_in', 100);
insert into object_attributes(object_id, attribute_name, attribute_value_object_id) values(17, 'is_in', 100);

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