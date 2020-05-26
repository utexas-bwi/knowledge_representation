from knowledge_representation._libknowledge_rep_wrapper_cpp import LongTermMemoryConduit, PyAttributeList, Entity, \
    EntityAttribute, Concept, Instance, AttributeValueType, Map, Point, Pose, Region


def get_default_ltmc():
    return LongTermMemoryConduit("knowledge_base")
