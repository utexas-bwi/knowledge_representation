from knowledge_representation._libknowledge_rep_wrapper_cpp import LongTermMemoryConduit, PyAttributeList, Entity, \
    Concept, Instance, AttributeValueType


def get_default_ltmc():
    return LongTermMemoryConduit("knowledge_base")
