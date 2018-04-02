#ifndef VILLA_WORLD_MODEL_WORLDMODELINTERFACE_H
#define VILLA_WORLD_MODEL_WORLDMODELINTERFACE_H


#include "LongTermMemoryConduit.h"
#include "ShortTermMemoryConduit.h"

namespace knowledge_rep {

    class MemoryConduit {
        LongTermMemoryConduit ltmc;
        ShortTermMemoryConduit stmc;

        explicit MemoryConduit(const std::string &ltmi_adress = "127.0.0.1") : ltmc(ltmi_adress, 33060, "root", "",
                                                                                    "villa_krr"), stmc() {

        }

    };

}

#endif //VILLA_WORLD_MODEL_WORLDMODELINTERFACE_H
