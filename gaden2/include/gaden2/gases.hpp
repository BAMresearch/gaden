#ifndef GADEN2_GASES_HPP_INCLUDED
#define GADEN2_GASES_HPP_INCLUDED

namespace gaden2::gases {

// TODO Temperature and pressure should be parameters

class GasBase
{
public:
    virtual double getMassDensity() const = 0; // [kg/m3], at 0° C, 1013 hPa
    virtual double getMolarMass() const = 0; // [kg/mol]
    virtual double getDynamicViscosity() const = 0; // [kg/(m*s)] = [Pa s], at 0° C, 1013 hPa
};

class Air : public GasBase
{
public:
    double getMassDensity() const { return 1.2920; } // [kg/m3], at 0° C, 1013 hPa
    double getMolarMass() const { return 28.9e-3; } // [kg/mol]
    double getDynamicViscosity() const { return 18.2e-6; } // [kg/(m*s)] = [Pa s], at 0° C, 1013 hPa
};

class Methane : public GasBase
{
public:
    double getMassDensity() const { return 0.72; } // [kg/m3], at 0° C, 1013 hPa
    double getMolarMass() const { return 16.043e-3; } // [kg/mol]
    double getDynamicViscosity() const { return 10.24e-6; } // [kg/(m*s)] = [Pa s], at 0° C, 1013 hPa
};

} // namespace gaden2::gases

#endif // GADEN2_GASES_HPP_INCLUDED
