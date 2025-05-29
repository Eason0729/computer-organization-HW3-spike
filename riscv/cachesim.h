// See LICENSE for license details.

#ifndef _RISCV_CACHE_SIM_H
#define _RISCV_CACHE_SIM_H

#include "memtracer.h"
#include "common.h"
#include <cstring>
#include <string>
#include <map>
#include <cstdint>

/**
 * @brief Linear Feedback Shift Register implementation for pseudorandom number generation
 * 
 * This class implements a Linear Feedback Shift Register (LFSR) that generates pseudorandom
 * numbers using a 32-bit polynomial. It's primarily used by the cache simulator for
 * implementing randomized cache replacement policies.
 */
class lfsr_t
{
 public:
  /**
   * Default constructor for Linear Feedback Shift Register
   * Initializes the register with a seed value of 1
   */
  lfsr_t() : reg(1) {}
  /**
   * @brief Copy constructor for Linear Feedback Shift Register
   *
   * @param lfsr Source LFSR to copy from
   *
   * Creates a new LFSR by copying the register value from an existing one
   */
  lfsr_t(const lfsr_t& lfsr) : reg(lfsr.reg) {}
  /**
   * @brief Generates the next pseudorandom number in the sequence
   *
   * @return uint32_t The next pseudorandom number
   *
   * Implements a 32-bit LFSR with polynomial 0xd0000001, shifting right and XORing
   * with the polynomial when the lowest bit is 1
   */
  uint32_t next() { return reg = (reg>>1)^(-(reg&1) & 0xd0000001); }
 private:
  uint32_t reg;
};

/**
 * @brief Generic cache simulator class
 *
 * This class implements a configurable cache simulator supporting different sizes,
 * associativities, and line sizes. It tracks cache statistics like hits, misses,
 * and writebacks, and can be used as a building block for more complex memory
 * hierarchies through the miss handler mechanism.
 */
class cache_sim_t
{
 public:
  /**
   * @brief Constructs a new cache simulator
   *
   * @param sets Number of cache sets
   * @param ways Number of ways (associativity)
   * @param linesz Size of cache line in bytes
   * @param name Name of the cache for reporting
   *
   * Creates a new cache with the specified organization parameters
   */
  cache_sim_t(size_t sets, size_t ways, size_t linesz, const char* name);
  /**
   * @brief Copy constructor for cache simulator
   *
   * @param rhs Source cache simulator to copy from
   *
   * Creates a new cache simulator by copying the configuration and state from an existing one
   */
  cache_sim_t(const cache_sim_t& rhs);
  /**
   * @brief Destructor for cache simulator
   *
   * Cleans up resources used by the cache simulator instance
   */
  virtual ~cache_sim_t();

  /**
   * @brief Simulates a cache access (read or write)
   *
   * @param addr Memory address to access
   * @param bytes Size of the access in bytes
   * @param store True if this is a write operation, false for read
   *
   * Simulates accessing the cache, updating hit/miss statistics and
   * handling cache line allocation and replacement as needed
   */
  void access(uint64_t addr, size_t bytes, bool store);
  void clean_invalidate(uint64_t addr, size_t bytes, bool clean, bool inval);
  void print_stats();
  void set_miss_handler(cache_sim_t* mh) { miss_handler = mh; }
  /**
   * @brief Enables or disables logging for this cache
   *
   * @param _log True to enable logging, false to disable
   *
   * Controls whether the cache simulator will produce detailed logs of its operations
   */
  void set_log(bool _log) { log = _log; }

  /**
   * @brief Factory method to construct a cache simulator from configuration string
   *
   * @param config String specifying cache parameters (e.g., "sets:ways:line_size")
   * @param name Name of the cache for reporting
   * @return cache_sim_t* Pointer to the newly created cache simulator
   *
   * Creates a cache simulator with the specified configuration. Returns nullptr if
   * the configuration string is invalid.
   */
  static cache_sim_t* construct(const char* config, const char* name);

 protected:
  static const uint64_t VALID = 1ULL << 63;
  static const uint64_t DIRTY = 1ULL << 62;

  /**
   * @brief Checks if an address is present in the cache
   *
   * @param addr Memory address to check
   * @return uint64_t* Pointer to the matching cache tag entry, or nullptr if not found
   *
   * Determines if the specified address hits in the cache and returns a pointer
   * to the corresponding tag entry if found
   */
  virtual uint64_t* check_tag(uint64_t addr);
  /**
   * @brief Selects and evicts a cache line for replacement
   *
   * @param addr Memory address for which a line needs to be allocated
   * @return uint64_t The memory address that was evicted (may trigger writeback)
   *
   * Implements the cache replacement policy by selecting a victim line to be
   * replaced when a new address needs to be cached
   */
  virtual uint64_t victimize(uint64_t addr);

  lfsr_t lfsr;
  cache_sim_t* miss_handler;

  size_t sets;
  size_t ways;
  size_t linesz;
  size_t idx_shift;

  uint64_t* tags;

  uint64_t read_accesses;
  uint64_t read_misses;
  uint64_t bytes_read;
  uint64_t write_accesses;
  uint64_t write_misses;
  uint64_t bytes_written;
  uint64_t writebacks;

  std::string name;
  bool log;

  void init();
};

/**
 * @brief Fully associative cache simulator
 * 
 * This class implements a fully associative cache where any block can go into any position
 * in the cache. It uses a map-based implementation for efficient tag lookup, and inherits
 * the basic functionality from the generic cache simulator class.
 */
class fa_cache_sim_t : public cache_sim_t
{
 public:
  /**
   * @brief Constructs a new fully associative cache simulator
   *
   * @param ways Number of ways (entries) in the cache
   * @param linesz Size of each cache line in bytes
   * @param name Identifier for this cache instance
   *
   * Creates a fully associative cache with the specified number of ways and line size
   */
  fa_cache_sim_t(size_t ways, size_t linesz, const char* name);
  /**
   * @brief Checks if an address is present in the fully associative cache
   *
   * @param addr Memory address to check
   * @return uint64_t* Pointer to the tag entry if present and valid, nullptr otherwise
   *
   * Overrides the base class method to implement tag checking for a fully associative cache
   */
  uint64_t* check_tag(uint64_t addr);
  /**
   * @brief Selects and evicts a cache line for replacement in the fully associative cache
   *
   * @param addr Memory address for which space is needed
   * @return uint64_t The tag value of the evicted cache line
   *
   * Implements the replacement policy for the fully associative cache
   */
  uint64_t victimize(uint64_t addr);
 private:
  /**
   * @brief Compares two cache tags
   *
   * @param a First tag to compare
   * @param b Second tag to compare
   * @return bool True if tags are equivalent, false otherwise
   *
   * Helper method used for tag comparison in the fully associative cache
   */
  static bool cmp(uint64_t a, uint64_t b);
  std::map<uint64_t, uint64_t> tags;
};

/**
 * @brief Memory tracer implementation using a cache simulator
 * 
 * This class connects the memory tracer interface to a cache simulator, allowing
 * cache simulations to be attached to memory accesses in the processor model.
 * It serves as a bridge between the memory tracing system and the cache simulation.
 */
class cache_memtracer_t : public memtracer_t
{
 public:
  /**
   * @brief Constructs a memory tracer that uses a cache simulator
   *
   * @param config String specifying cache parameters
   * @param name Identifier for the cache instance
   *
   * Creates a memory tracer that routes memory accesses through a cache simulator
   */
  cache_memtracer_t(const char* config, const char* name)
  {
    cache = cache_sim_t::construct(config, name);
  }
  /**
   * @brief Destroys the cache memory tracer
   *
   * Cleans up resources, including the owned cache simulator
   */
  ~cache_memtracer_t()
  {
    delete cache;
  }
  void set_miss_handler(cache_sim_t* mh)
  {
    cache->set_miss_handler(mh);
  }
  void clean_invalidate(uint64_t addr, size_t bytes, bool clean, bool inval)
  {
    cache->clean_invalidate(addr, bytes, clean, inval);
  }
  /**
   * @brief Sets the logging state of the underlying cache simulator
   *
   * @param log Whether logging should be enabled (true) or disabled (false)
   *
   * Controls whether the underlying cache simulator logs its operations
   */
  void set_log(bool log)
  {
    cache->set_log(log);
  }
  void print_stats()
  {
    cache->print_stats();
  }

 protected:
  cache_sim_t* cache;
};

/**
 * @brief Instruction cache simulator
 * 
 * This class implements a specialized cache simulator for instruction fetches.
 * It filters memory accesses to only process instruction fetches, ignoring data
 * load and store operations.
 */
class icache_sim_t : public cache_memtracer_t
{
 public:
  /**
   * @brief Constructs a new instruction cache simulator
   *
   * @param config String specifying cache parameters
   * @param name Identifier for this cache instance (defaults to "I$")
   *
   * Creates an instruction cache simulator with the specified parameters
   */
  icache_sim_t(const char* config, const char* name = "I$")
	  : cache_memtracer_t(config, name) {}
  /**
   * @brief Determines if this cache is interested in a given memory access range
   *
   * @param UNUSED begin Start address of the range (unused)
   * @param UNUSED end End address of the range (unused)
   * @param type Type of memory access
   * @return bool True if this is a fetch access, false otherwise
   *
   * Instruction cache is only interested in instruction fetches
   */
  /**
   * @brief Determines if this cache is interested in a given memory access range
   *
   * @param UNUSED begin Start address of the range (unused)
   * @param UNUSED end End address of the range (unused)
   * @param type Type of memory access
   * @return bool True if this is a load or store access, false otherwise
   *
   * Data cache is only interested in load and store accesses
   */
  bool interested_in_range(uint64_t UNUSED begin, uint64_t UNUSED end, access_type type)
  {
    return type == FETCH;
  }
  /**
   * @brief Traces a memory access through the instruction cache
   *
   * @param addr Memory address being accessed
   * @param bytes Number of bytes being accessed
   * @param type Type of memory access
   *
   * Simulates an instruction fetch through the cache if the access type is FETCH
   */
  /**
   * @brief Traces a memory access through the data cache
   *
   * @param addr Memory address being accessed
   * @param bytes Number of bytes being accessed
   * @param type Type of memory access
   *
   * Simulates a data access through the cache if the access type is LOAD or STORE
   */
  void trace(uint64_t addr, size_t bytes, access_type type)
  {
    if (type == FETCH) cache->access(addr, bytes, false);
  }
};

/**
 * @brief Data cache simulator
 * 
 * This class implements a specialized cache simulator for data accesses.
 * It filters memory accesses to only process loads and stores, ignoring
 * instruction fetches.
 */
class dcache_sim_t : public cache_memtracer_t
{
 public:
  /**
   * @brief Constructs a new data cache simulator
   *
   * @param config String specifying cache parameters
   * @param name Identifier for this cache instance (defaults to "D$")
   *
   * Creates a data cache simulator with the specified parameters
   */
  dcache_sim_t(const char* config, const char* name = "D$")
	  : cache_memtracer_t(config, name) {}
  bool interested_in_range(uint64_t UNUSED begin, uint64_t UNUSED end, access_type type)
  {
    return type == LOAD || type == STORE;
  }
  void trace(uint64_t addr, size_t bytes, access_type type)
  {
    if (type == LOAD || type == STORE) cache->access(addr, bytes, type == STORE);
  }
};

#endif
