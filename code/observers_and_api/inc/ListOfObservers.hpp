/*
 * ListOfObservers.hpp
 *
 *  Created on: Jan 12, 2015
 *      Author: cady
 */

#ifndef LISTOFOBSERVERS_HPP_
#define LISTOFOBSERVERS_HPP_

#include <ssc/macros.hpp>
#include TR1INC(memory)

#include "Observer.hpp"

typedef TR1(shared_ptr)<Observer> ObserverPtr;

struct YamlOutput;

class ListOfObservers
{
    public:
        ListOfObservers(const std::vector<YamlOutput>& yaml);
        void observe(const Sim& sys, const double t);
        std::vector<ObserverPtr> get() const;

        template <typename T> void write(
                const T& val,
                const DataAddressing& address)
        {
            for (auto observer:observers)
            {
                observer->write(val, address);
            }
        }

    private:

        std::vector<ObserverPtr> observers;
};

#endif /* LISTOFOBSERVERS_HPP_ */
