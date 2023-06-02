#pragma once

#include <map>

#include "a_star.hpp"
#include "timer.hpp"
#include <boost/heap/priority_queue.hpp>
#include <thread>

#include <queue>
#include <mutex>
#include <condition_variable>

template<typename T>
class LockingQueue
{
public:

    size_t size(){
        return queue.size();
    }

    void push(T const& _data)
    {
        {
            std::lock_guard<std::mutex> lock(guard);
            queue.push(_data);
        }
        signal.notify_one();
    }

    bool empty() const
    {
        std::lock_guard<std::mutex> lock(guard);
        return queue.empty();
    }

    bool tryPop(T& _value)
    {
        std::lock_guard<std::mutex> lock(guard);
        if (queue.empty())
        {
            return false;
        }

        _value = queue.front();
        queue.pop();
        return true;
    }

    void waitAndPop(T& _value)
    {
        std::unique_lock<std::mutex> lock(guard);
        while (queue.empty())
        {
            signal.wait(lock);
        }

        _value = queue.top();
        queue.pop();
    }

    bool tryWaitAndPop(T& _value, int _milli)
    {
        std::unique_lock<std::mutex> lock(guard);
        while (queue.empty())
        {
            signal.wait_for(lock, std::chrono::milliseconds(_milli));
            return false;
        }

        _value = queue.front();
        queue.pop();
        return true;
    }

private:
    std::priority_queue<T> queue;
    mutable std::mutex guard;
    std::condition_variable signal;
};


// #define USEHEAP

template <typename State, typename Action, typename Cost, typename Conflict,
          typename Constraints, typename Environment>
class CBS {
 public:
    CBS(Environment& environment,
        std::vector<State>& aInitialStates,
        std::vector<PlanResult<State, Action, Cost> >& aSolution,
        const int aThreadsNum) : solution{aSolution},
        m_env(environment),
        initialStates{aInitialStates},
        solutionFound{false},
        idNode{0},
        threadsNum {aThreadsNum}
  {}

  bool search(/*std::vector<State>& aInitialStates,*/
              /*std::vector<PlanResult<State, Action, Cost> >& solution*/) {

    Timer timer2;

    HighLevelNode start;
    start.solution.resize(initialStates.size());
    start.constraints.resize(initialStates.size());
    start.cost = 0;
    start.id = 0;

    idNode++;

    for (size_t i = 0; i < initialStates.size(); ++i) {
      // if (   i < solution.size()
      //     && solution[i].states.size() > 1) {
      //   start.solution[i] = solution[i];
      //   std::cout << "use existing solution for agent: " << i << std::endl;
      // } else {
      LowLevelEnvironment llenv(m_env, i, start.constraints[i]);
      LowLevelSearch_t lowLevel(llenv);
      bool success = lowLevel.search(initialStates[i], start.solution[i]);
      if (!success) {
        return false;
      }
      // }
      start.cost += start.solution[i].cost;
    }

    timer2.stop();
    // std::cout << "  Optimal trajectories runtime: " << timer2.elapsedSeconds() << std::endl;

#ifndef USEHEAP
    open.push(start);
#else
    auto handle = open.push(start);
    (*handle).handle = handle;
#endif

    solution.clear();

    //bool result = NodeCheck(initialStates, solution);
    std::cout << "NumberOfThreads: " << threadsNum << std::endl;

    if (threadsNum > 1) {
        std::vector<std::thread> threadsPool;
        for (int i = 0; i<threadsNum; ++i){
            threadsPool.emplace_back(std::thread(&CBS::NodeCheck, this, i));
        }

        // Wait for all threads to finish
        for (auto& thread : threadsPool) {
            thread.join();
        }
        // std::cout << "solution size from main: " << solution.size() << std::endl;
        // std::cout << "All threads finished" << std::endl;
    } else {
        NodeCheck(0);
    }
    return true;
  }

  bool NodeCheck(int id){
      while (true){

          if (solutionFound){
            break;
          }
          HighLevelNode P;
          //if (open.empty()){
          //    continue;
          //} else {
//              std::unique_lock<std::mutex> lock(mutex);
//              if (!open.empty()){
                  // std::cout << "Thr id " << id << " Before POP: Size of heap: " << open.size() << std::endl;
                  open.waitAndPop(P);
                  //P = open.top();
                  //open.pop();
                  //lock.unlock();
              //} else {
              //    continue;
              //}
          //}

            // std::cout << "Thr id " << id << " Size of heap: " << open.size() << std::endl;

            m_env.onExpandHighLevelNode(P.cost);

            // std::cout << "id: " << id << " expand: " << P.id << std::endl;
            // std::cout << "*****id: " << id << " 2 " << std::endl;

            Conflict conflict;
            if (!m_env.getFirstConflict(P.solution, conflict)) {
              std::cout << "done; cost: " << P.cost << std::endl;
              if (!solutionFound){
                  solutionFound = true;
                  solution = P.solution;
                  // std::cout << "solution size from thread: " << solution.size() << std::endl;
              } else {
                  break;
              }
              // std::cout << "solution size from thread: " << solution.size() << std::endl;
              return true;
            }

            // create additional nodes to resolve conflict
            // std::cout << "Found conflict: " << conflict << std::endl;
            // std::cout << "Found conflict at t=" << conflict.time << " type: " <<
            // conflict.type << std::endl;

            // std::cout << "*****id: " << id << " 3 " << std::endl;

            std::map<size_t, Constraints> constraints;
            m_env.createConstraintsFromConflict(conflict, constraints);
            for (const auto& c : constraints) {
              // std::cout << "Add HL node for " << c.first << std::endl;
              size_t i = c.first;
              //std::cout << "create child with id " << id << std::endl;
              HighLevelNode newNode = P;
              newNode.id = ++idNode;
              // (optional) check that this constraint was not included already
              // std::cout << "id: " << id << " NewNode: " << newNode.constraints[i] << std::endl;
              // std::cout << "id: " << id << " second: " << c.second << std::endl;
              assert(!newNode.constraints[i].overlap(c.second));

              newNode.constraints[i].add(c.second);

              newNode.cost -= newNode.solution[i].cost;

              LowLevelEnvironment llenv(m_env, i, newNode.constraints[i]);
              LowLevelSearch_t lowLevel(llenv);
              bool success = lowLevel.search(initialStates[i], newNode.solution[i]);

              newNode.cost += newNode.solution[i].cost;

              // std::cout << "*****id: " << id << " 4 " << std::endl;

              if (success) {
              // std::cout << "*****id: " << id << " Before push " << std::endl;
                // std::cout << "  success. cost: " << newNode.cost << std::endl;

        #ifndef USEHEAP
                open.push(newNode);
        #else
                auto handle = open.push(newNode);
                (*handle).handle = handle;
        #endif
              // std::cout << "*****id: " << id << " After push " << std::endl;
              // std::cout << "**********id: " << id << "       SUCCESS " << std::endl;
              } else {
                // std::cout << "**********id: " << id << " NOT SUCCESS " << std::endl;
              }
            }

            // std::cout << "*****id: " << id << " 5 " << std::endl;

            // std::cout << "Size of heap: " << open.size() << std::endl;
          }
  }
 private:
  struct HighLevelNode {
    std::vector<PlanResult<State, Action, Cost> > solution;
    std::vector<Constraints> constraints;

    Cost cost;

    int id;

#ifdef USEHEAP
    typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
                                     boost::heap::mutable_<true> >::handle_type
        handle;
#endif

    bool operator<(const HighLevelNode& n) const {
      // if (cost != n.cost)
      return cost > n.cost;
      // return id > n.id;
    }

    friend std::ostream& operator<<(std::ostream& os, const HighLevelNode& c) {
      os << "id: " << c.id << " cost: " << c.cost << std::endl;
      for (size_t i = 0; i < c.solution.size(); ++i) {
        os << "Agent: " << i << std::endl;
        os << " States:" << std::endl;
        for (size_t t = 0; t < c.solution[i].states.size(); ++t) {
          os << "  " << c.solution[i].states[t].first << std::endl;
        }
        os << " Constraints:" << std::endl;
        os << c.constraints[i];
        os << " cost: " << c.solution[i].cost << std::endl;
      }
      return os;
    }
  };

  struct LowLevelEnvironment {
    LowLevelEnvironment(Environment& env, size_t agentIdx,
                        const Constraints& constraints)
        : m_env(env)
    // , m_agentIdx(agentIdx)
    // , m_constraints(constraints)
    {
      m_env.setLowLevelContext(agentIdx, &constraints);
    }

    Cost admissibleHeuristic(const State& s) {
      return m_env.admissibleHeuristic(s);
    }

    bool isSolution(const State& s) { return m_env.isSolution(s); }

    void getNeighbors(const State& s,
                      std::vector<Neighbor<State, Action, Cost> >& neighbors) {
      m_env.getNeighbors(s, neighbors);
    }

    void onExpandNode(const State& s, Cost fScore, Cost gScore) {
      // std::cout << "LL expand: " << s << std::endl;
      m_env.onExpandLowLevelNode(s, fScore, gScore);
    }

    void onDiscover(const State& /*s*/, Cost /*fScore*/, Cost /*gScore*/) {
      // std::cout << "LL discover: " << s << std::endl;
      // m_env.onDiscoverLowLevel(s, m_agentIdx, m_constraints);
    }

   private:
    Environment m_env;
    // size_t m_agentIdx;
    // const Constraints& m_constraints;
  };

 private:

#ifndef USEHEAP
    LockingQueue<HighLevelNode> open;
#else
    typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
                                     boost::heap::mutable_<true> >
        open;
#endif

  std::mutex mutex;
  std::vector<PlanResult<State, Action, Cost> >& solution;
  std::vector<State>& initialStates;
  bool solutionFound;
  Environment& m_env;
  typedef AStar<State, Action, Cost, LowLevelEnvironment> LowLevelSearch_t;
  int idNode;
  int threadsNum;
};
