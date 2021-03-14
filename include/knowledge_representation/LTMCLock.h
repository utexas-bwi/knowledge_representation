#pragma once
#include <utility>
#include <vector>
#include <string>

#include <knowledge_representation/LTMCConcept.h>
#include <knowledge_representation/LTMCEntity.h>

namespace knowledge_rep
{
/// \brief Represents a lock on the backing database.

/// Currently, there is no granularity in locking. Holding the lock will prevent all other knowledge_rep clients
/// from performing any operations on the database.
template <typename LTMCImpl>
class LTMCLock
{
  friend LTMCImpl;

public:
  std::reference_wrapper<LongTermMemoryConduitInterface<LTMCImpl>> ltmc;

  /**
   * Constructs and optionally acquires a lock
   *
   * Currently, there is no granularity in locking, and therefore only a single type of lock.
   *
   * @param inLtmc The LTMC to lock
   * @param lock whether to lock immediately
   */
  LTMCLock(LongTermMemoryConduitInterface<LTMCImpl>& inLtmc, bool lock) : ltmc(inLtmc)
  {
    if (lock)
    {
      ltmc.get().acquireLock(*this);
    }
  }

  /**
   * Destroys the lock object and releases the lock if it is held
   */
  ~LTMCLock()
  {
    ltmc.get().releaseLock(*this);
  }

  /**
   * Makes a blocking attempt to acquire the lock
   *
   * Currently, there is no granularity in locking. Holding the lock will prevent all other knowledge_rep clients
   * from performing any operations on the database.
   *
   * @return true if the lock was acquired
   */
  bool acquire()
  {
    return ltmc.get().acquireLock(*this);
  }

  /**
   * Releases the lock if it is held
   * @return true if the lock was held and it was released
   */
  bool release()
  {
    return ltmc.get().releaseLock(*this);
  }
};

}  // namespace knowledge_rep
