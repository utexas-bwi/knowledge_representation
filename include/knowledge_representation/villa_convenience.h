#ifndef KNOWLEDGE_REPRESENTATION_CONVENIENCE_H
#define KNOWLEDGE_REPRESENTATION_CONVENIENCE_H

#include "LongTermMemoryConduit.h"

namespace knowledge_rep {
    namespace villa {
        LongTermMemoryConduit get_default_ltmc() {
            return LongTermMemoryConduit("127.0.0.1", 33060, "root", "", "villa_krr");
        }
    }
}
#endif //KNOWLEDGE_REPRESENTATION_CONVENIENCE_H
