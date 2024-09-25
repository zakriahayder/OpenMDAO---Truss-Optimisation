import openmdao.api as om
import numpy as np

class Truss(om.ExplicitComponent):
    """_summary_

    Args:
        om (_type_): _description_
    """

    def setup(self):

        self.add_input('H', val=30)
        self.add_input('d', val=3)
        #constants
        self.add_input('B', val=60)
        self.add_input('t', val=0.15, desc= 'thickness')
        self.add_input('E', val=30000, desc= 'modulus')
        self.add_input('P', val=60, desc='load')
        self.add_input('rho', val=0.3)
        self.add_input('stress_max', val=100)
        self.add_input('d_max', val = 0.25, desc= 'max allowable deflection')
        #outputs
        self.add_output('cost', val=0)
        self.add_output('stress', val=0)
        self.add_output('buckling', val=0)
        self.add_output('delfection', val = 0)

    def setup_partials(self):
        self.declare_partials('*', '*', method = 'fd')

    def compute(self, inputs, outputs):
        H = inputs['H']
        d = inputs['d']
        B = inputs['B']
        t = inputs['t']
        E = inputs['E']
        P = inputs['P']
        rho = inputs['rho']

        L = np.sqrt((B/2)**2 + H**2)
        A = np.pi * d * t
        IoverA = (d ** 2 + t **2) / 8

        stress = (P * L) / (2 * A * H)
        buckling = (np.pi ** 2) * E * IoverA / (L ** 2)
        cost = 2 * rho * A * L
        deflection = P * L ** 3 / (2 * E * A * H ** 2)

        outputs['stress'] = stress
        outputs['deflection'] = deflection
        outputs['buckling'] = buckling
        outputs['cost'] = cost

prob = om.Problem()
prob.model.add_subsystem('truss', Truss())

prob.driver = om.ScipyOptimizeDriver()
prob.driver.options['optimizer'] = 'SLSQP'

prob.model.add_design_var('truss.H', lower = 10, upper = 30)
prob.model.add_design_var('truss.d', lower = 1.0, upper = 3.0)

prob.model.add_objective('truss.cost')

prob.model.add_constraint('truss.stress', upper = 100)
prob.model.add_constraint('truss.deflection', upper = 0.25)

prob.model.add_subsystem('buckling_constraint', om.ExecComp('stress_minus_buckling = stress - buckling'))
prob.model.connect('truss.stress', 'buckling_constraint.stress')
prob.model.connect('truss.buckling', 'buckling_constraint.buckling')
prob.model.add_constraint('buckling_constraint.F', upper = 0)

prob.setup()

prob.set_val('truss.H', 30)
prob.set_val('truss.d', 3.0)
prob.set_val('truss.B', 60.0)
prob.set_val('truss.t', 0.15)
prob.set_val('truss.E', 30000.0)
prob.set_val('truss.P', 66.0)
prob.set_val('truss.rho', 0.3)

prob.run_driver()
print("\nOptimal Design Variables:")
print(f"Optimal Height (H): {prob.get_val('truss.H')[0]:.4f}")
print(f"Optimal Diameter (d): {prob.get_val('truss.d')[0]:.4f}")

print("\nObjective Function:")
print(f"Minimum Cost: {prob.get_val('truss.cost')[0]:.4f}")

print("\nConstraints:")
print(f"Stress: {prob.get_val('truss.stress')[0]:.4f}")
print(f"Buckling Stress: {prob.get_val('truss.buckling')[0]:.4f}")
print(f"Deflection: {prob.get_val('truss.deflection')[0]:.4f}")
