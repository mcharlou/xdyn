/*
 * History.hpp
 *
 *  Created on: Nov 26, 2014
 *      Author: cady
 */

#ifndef HISTORY_HPP_
#define HISTORY_HPP_

/** \brief
 *  \details
 *  \addtogroup hdb_interpolators
 *  \ingroup hdb_interpolators
 *  \section ex1 Example
 *  \snippet hdb_interpolators/unit_tests/src/HistoryTest.cpp HistoryTest example
 *  \section ex2 Expected output
 *  \snippet hdb_interpolators/unit_tests/src/HistoryTest.cpp HistoryTest expected output
 */
class History
{
    public:
        History(const double Tmax //!< Maximum duration to store in history (in seconds)
               );

        /**  \brief Returns the value at t-tau, t being the current instant
          *  \returns Value at t-tau in history
          *  \snippet hdb_interpolator/unit_tests/src/HistoryTest.cpp HistoryTest get_example
          */
        double get(double tau //!< How far back in history do we need to go (in seconds)?
                               ) const;

    private:
        History(); // Disabled
        double Tmax;
};


#endif /* HISTORY_HPP_ */
