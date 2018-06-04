#include <knowledge_representation/villa_convenience.h>

namespace knowledge_rep {
    namespace villa {
        LongTermMemoryConduit get_default_ltmc() {
            return LongTermMemoryConduit("127.0.0.1", 33060, "root", "", "villa_krr");
        }
    }
}
