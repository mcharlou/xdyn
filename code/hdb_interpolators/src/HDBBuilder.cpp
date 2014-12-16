/*
 * HDBBuilder.cpp
 *
 *  Created on: Nov 13, 2014
 *      Author: cady
 */

#include <list>
#include <set>
#include <sstream>

#include <boost/lexical_cast.hpp>

#include "hdb_parser_internal_data_structures.hpp"
#include "HDBBuilder.hpp"
#include "HDBBuilderException.hpp"
#include "hdb_to_ast.hpp"

class HDBBuilder::Impl
{
    public:
        Impl(const std::string& data) : tree(hdb::parse(data))
        {
        }

        void fill(TimestampedMatrices& ret, const size_t i, const hdb::ListOfValues& M) const
        {
            if (ret.empty()) ret.resize(M.size());
            if (M.size() != ret.size())
            {
                std::stringstream ss;
                ss << "Something is wrong with the HDB file: detected "
                   << ret.size()
                   << " periods in Added_mass_Radiation_Damping, but line "
                   << i << " has " << M.size() << " periods";
                THROW(__PRETTY_FUNCTION__, HDBBuilderException,  ss.str());
            }
            for (size_t k = 0 ; k < M.size() ; ++k)
            {
                ret[k].first = M.at(k).front();
                for (size_t j = 0 ; j < 6 ; ++j)
                {
                    ret[k].second[i][j] = M.at(k).at(j+1);
                }
            }
        }

        TimestampedMatrices get_matrix(const std::string& header, const std::string& matrix) const
        {
            TimestampedMatrices ret;
            std::vector<bool> found_line(6,false);
            for (auto M = tree.lists_of_matrix_sections.begin() ; M != tree.lists_of_matrix_sections.end() ; ++M)
            {
                if (M->header == header)
                {
                    for (auto that_section = M->sections.begin() ; that_section != M->sections.end() ; ++that_section)
                    {
                        for (size_t i = 0 ; i < 6 ; ++i)
                        {
                            if (that_section->header == matrix + "_" + boost::lexical_cast<std::string>(i+1)) fill(ret, i, that_section->values); found_line[i] = true;
                        }
                    }
                }
            }
            for (size_t i = 0 ; i < 6 ; ++i)
            {
                if (not(found_line[i]))
                {
                    std::stringstream ss;
                    ss << "Unable to find key '" << matrix << "_" << i+1 << "' in HDB file";
                    THROW(__PRETTY_FUNCTION__, HDBBuilderException, ss.str());
                }
            }
            return ret;
        }

        RAOData get_rao(const std::string& section_name, const std::string& subsections) const
        {
            std::set<double> omegas, psi;
            RAOData ret;
            for (auto ms:tree.lists_of_matrix_sections_with_id)
            {
                if (ms.header == section_name)
                {
                    for (auto s:ms.sections_with_id)
                    {
                        if (s.header == subsections)
                        {
                            psi.insert(s.id);
                            std::array<std::vector<double>,6> columns;
                            for (auto v:s.values)
                            {
                                omegas.insert(v.front());
                                for (size_t j = 0 ; j < 6 ; ++j)
                                {
                                    columns.at(j).push_back(v.at(j+1));
                                }
                            }
                            for (size_t j = 0 ; j < 6 ; ++j)
                            {
                                ret.values.at(j).push_back(columns.at(j));
                            }
                        }
                    }
                }
            }
            std::list<double> omega_l(omegas.begin(), omegas.end());
            omega_l.sort();
            ret.omega.insert(ret.omega.begin(),omega_l.begin(), omega_l.end());
            std::list<double> psi_l(psi.begin(), psi.end());
            omega_l.sort();
            ret.psi.insert(ret.psi.begin(),psi_l.begin(), psi_l.end());
            return ret;
        }

        RAOData get_diffraction_module() const
        {
            return get_rao("FROUDE-KRYLOV_FORCES_AND_MOMENTS", "INCIDENCE_EFM_MOD_001");
        }

        RAOData get_diffraction_phase() const
        {
            return get_rao("FROUDE-KRYLOV_FORCES_AND_MOMENTS", "INCIDENCE_EFM_PH_001");
        }

        hdb::AST tree;
};


HDBBuilder::HDBBuilder(const std::string& data) : pimpl(new Impl(data))
{
}

TimestampedMatrices HDBBuilder::get_added_mass() const
{
    return pimpl->get_matrix("Added_mass_Radiation_Damping", "ADDED_MASS_LINE");
}

TimestampedMatrices HDBBuilder::get_radiation_damping() const
{
    return pimpl->get_matrix("Added_mass_Radiation_Damping", "DAMPING_TERM");
}

RAOData HDBBuilder::get_diffraction_module() const
{
    return pimpl->get_diffraction_module();
}

RAOData HDBBuilder::get_diffraction_phase() const
{
    return pimpl->get_diffraction_phase();
}
