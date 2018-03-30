drop database if exists villa_krr;
create database villa_krr;
use villa_krr;

create table objects (
    object_id int not null,
    primary key(object_id)
);

create table attributes (
    attribute_name varchar(24) not null,
    primary key(attribute_name)
);

create table object_attributes (
    object_id int not null,
    attribute_name varchar(24) not null,
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


create table object_types (
    type_name varchar(24),
    primary key (type_name)
);

create table attribute_types (
    attribute_name varchar(24),
    primary key(attribute_name)
);

create table is_a (
    more_specific varchar(24),
    less_specific varchar(24),
    primary key (more_specific, less_specific),
    foreign key(more_specific) references object_types(type_name),
    foreign key(less_specific) references object_types(type_name)
);

create table known_object_weights (
    type_name varchar(24),
    weight float,
    primary key (type_name)
);

create table location (
    object_id int not null,
    location_id int not null,
    is_in boolean,
    is_on boolean,
    primary key(object_id, location_id),
    foreign key(object_id) references objects(id),
    foreign key(location_id) references objects(id)
);

create table person (
    person_id int not null,
    primary key(person_id)
);

create table object_attributes (
    object_id int not null,
    attribute_name varchar(24),
    primary key(object_id, attribute_name)
);

create table type_attributes (
    type_name varchar(24),
    attribute_name varchar(24),
    primary key(type_name, attribute_name)
);

create table person_attributes (
    person_id int not null,
    attribute_name varchar(24),
    primary key(person_id, attribute_name)
);

insert into object_types(type_name) values('unknown');
insert into object_types(type_name) values('unknown_surface');
insert into object_types(type_name) values('table');
insert into object_types(type_name) values('surface');
insert into object_types(type_name) values('shelf');
insert into object_types(type_name) values('drink');
insert into object_types(type_name) values('coke');
insert into object_types(type_name) values('coke_can');
insert into object_types(type_name) values('coke_bottle');

insert into is_a(more_specific, less_specific) values('table', 'surface');
insert into is_a(more_specific, less_specific) values('coke', 'drink');
insert into is_a(more_specific, less_specific) values('coke_can', 'coke');
insert into is_a(more_specific, less_specific) values('coke_bottle', 'coke');

insert into object_types(type_name) values('room');
insert into object_types(type_name) values('unknown_room');
insert into object_types(type_name) values('kitchen');
insert into object_types(type_name) values('dining room');
insert into object_types(type_name) values('living room');

