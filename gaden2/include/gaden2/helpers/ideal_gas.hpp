#ifndef GADEN2_HELPERS_IDEAL_GAS_HPP_INCLUDED
#define GADEN2_HELPERS_IDEAL_GAS_HPP_INCLUDED

namespace gaden2 {
namespace constants {

// gravity
static constexpr double g = 9.81; // [m/s2]

// gas constant
static constexpr double R = 8.31446261815324; // [m3 Pa / (K * mol)]

// Avogadro constant
static constexpr double N_Avogadro = 6.02214076e23; // [1/mol]

} // namespace constants, still in namespace gaden2

namespace ideal_gas {

/** Temperature **/
inline constexpr double toKelvinFromDegreeCelsius(double degree_celsius)
{
    return degree_celsius + 273.15;
}

/** Pressure **/
inline constexpr double toPascalFromAtmosphere(double atm)
{
    return atm * 101325.0;
}

inline constexpr double toPascalFromBar(double bar)
{
    return bar * 1e5;
}

/**
 * @brief getMolarFraction
 * @param gas_molar_concentration [mol/m3]
 * @param temperature [K]
 * @param pressure [Pa]
 * @return The amount of the gas divided by the total amount of all
 *         constituents in an ideal gas (dimensionless, []).
 */
constexpr double getMolarFraction(double gas_molar_concentration,
                                  double temperature,
                                  double pressure)
{
    const double ideal_gas_m3_per_mol = constants::R * temperature / pressure;
    //                       [m3/mol] = [m3 Pa / (K * mol)] * [K] / [Pa]

    return gas_molar_concentration * ideal_gas_m3_per_mol;
}

/**
 * @brief getMolarFractionAtNormalConditions
 *        Compute molar fraction at normal conditions (0° C, 1 atm)
 * @param gas_molar_concentration in [mol/m3]
 * @return The amount of the gas divided by the total amount of all
 *         constituents in an ideal gas (dimensionless, []).
 */
double getMolarFractionAtNormalConditions(double gas_molar_concentration)
{
    static constexpr double IDEAL_GAS_M3_PER_MOL =
            constants::R *                 // [m3 Pa / (K * mol)] *
            toKelvinFromDegreeCelsius(0) / // [K] /
            toPascalFromAtmosphere(1);     // [Pa] = [m3/mol]

    //static constexpr double IDEAL_GAS_MOLAR_CONCENTRATION =
    //        1.0 / IDEAL_GAS_M3_PER_MOL;              // [mol/m3]

    //return gas_molar_concentration / IDEAL_GAS_MOLAR_CONCENTRATION; // []
    return gas_molar_concentration * IDEAL_GAS_M3_PER_MOL; // []
}

} // namespace ideal_gas, still in namespace gaden2
} // namespace gaden2

#endif // GADEN2_HELPERS_IDEAL_GAS_HPP_INCLUDED
