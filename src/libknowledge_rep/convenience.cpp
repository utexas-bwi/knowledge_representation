#include <knowledge_representation/convenience.h>
#include <string>

namespace knowledge_rep
{

LongTermMemoryConduit getDefaultLTMC()
{
  std::string db_name = "knowledge_base";
  std::string db_pass = "nopass";
  if (const char *env_db_name = std::getenv("KNOWLEDGE_REP_DB_NAME"))
  {
    db_name = env_db_name;
  }
  if (const char *env_db_pass = std::getenv("KNOWLEDGE_REP_DB_PASSWORD"))
  {
    db_name = env_db_pass;
  }
  return LongTermMemoryConduit(db_name);
}

}  // namespace knowledge_rep
