import numpy as np
import openmdao.api as om

class TrussAnalysis(om.ExplicitComponent):
    def setup(self):
        # Inputs
        self.add_input('H', val=30.0)
        self.add_input('d', val=3.0)

        # Constants
        self.add_input('B', val=60.0)
        self.add_input('t', val=0.15)
        self.add_input('E', val=30000.0)
        self.add_input('P', val=66.0)
        self.add_input('rho', val=0.3)
        self.add_input('stressMax', val=100.0)
        self.add_input('deflectionMax', val=0.25)

        # Outputs
        self.add_output('cost', val=0.0)
        self.add_output('stress', val=0.0)
        self.add_output('buckling', val=0.0)
        self.add_output('deflection', val=0.0)

        # Declare partial derivatives
        self.declare_partials('*', '*', method='fd')

    def compute(self, inputs, outputs):
        H = inputs['H']
        d = inputs['d']
        B = inputs['B']
        t = inputs['t']
        E = inputs['E']
        P = inputs['P']
        rho = inputs['rho']

        L = np.sqrt((B / 2) ** 2 + H ** 2)
        A = np.pi * d * t
        IoverA = (d ** 2 + t ** 2) / 8

        stress = P * L / (2 * A * H)
        deflection = P * L ** 3 / (2 * E * A * H ** 2)
        buckling = (np.pi ** 2) * E * IoverA / (L ** 2)
        cost = 2 * rho * A * L

        outputs['stress'] = stress
        outputs['deflection'] = deflection
        outputs['buckling'] = buckling
        outputs['cost'] = cost

def run_truss_optimization():
    prob = om.Problem()
    model = prob.model

    # Add independent variables
    indeps = model.add_subsystem('indeps', om.IndepVarComp(), promotes=['*'])
    indeps.add_output('H', val=30.0)
    indeps.add_output('d', val=3.0)
    indeps.add_output('B', val=60.0)
    indeps.add_output('t', val=0.15)
    indeps.add_output('E', val=30000.0)
    indeps.add_output('P', val=66.0)
    indeps.add_output('rho', val=0.3)
    indeps.add_output('stressMax', val=100.0)
    indeps.add_output('deflectionMax', val=0.25)

    # Add the analysis component
    model.add_subsystem('truss', TrussAnalysis(), promotes=['*'])

    # Define the design variables
    model.add_design_var('H', lower=10.0, upper=30.0)
    model.add_design_var('d', lower=1.0, upper=3.0)

    # Objective is to minimize cost
    model.add_objective('cost')

    # Constraints using alias
    model.add_constraint('stress', upper=100.0, alias='stress_constraint')  # stress <= 100

    # To express stress <= buckling, we create a new variable stress_minus_buckling and constrain it
    class StressBucklingConstraint(om.ExplicitComponent):
        def setup(self):
            self.add_input('stress')
            self.add_input('buckling')
            self.add_output('stress_minus_buckling')

            self.declare_partials('*', '*', method='fd')

        def compute(self, inputs, outputs):
            outputs['stress_minus_buckling'] = inputs['stress'] - inputs['buckling']

    model.add_subsystem('stress_buckling_con', StressBucklingConstraint(), promotes=['*'])
    model.add_constraint('stress_minus_buckling', upper=0.0, alias='stress_buckling_constraint')  # stress - buckling <= 0

    # Deflection constraint
    model.add_constraint('deflection', upper=0.25)  # deflection <= deflectionMax

    # Setup the problem
    prob.setup()

    # Set the optimizer
    prob.driver = om.ScipyOptimizeDriver()
    prob.driver.options['optimizer'] = 'SLSQP'
    prob.driver.options['disp'] = True

    # Run the optimization
    prob.run_driver()

    # Print the results
    print("\nOptimal Design Variables:")
    print("Optimal Height (H):", prob.get_val('H')[0])
    print("Optimal Diameter (d):", prob.get_val('d')[0])
    print("\nObjective Function:")
    print("Minimum Cost:", prob.get_val('cost')[0])
    print("\nConstraints:")
    print("Stress:", prob.get_val('stress')[0])
    print("Buckling Stress:", prob.get_val('buckling')[0])
    print("Deflection:", prob.get_val('deflection')[0])

if __name__ == "__main__":
    run_truss_optimization()
