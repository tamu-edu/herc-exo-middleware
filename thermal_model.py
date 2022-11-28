import numpy as np

# The following parameters result from Jack Schuchmann's test with no fans
default_thermal_parameters=dict(
	C_w =  0.20*81.46202695970649,
	R_WC =  1.0702867186480716,
	C_c =  512.249065845453,
	R_CA =  1.9406620046327363,
	α = 0.393*1/100, #Pure copper. Taken from thermalmodel3.py
	R_T_0 = 65,# temp at which resistance was measured
	R_ϕ_0 = .376 # emirical, from the computed resistance (q-axis voltage/ q-axis current). Ohms
	)

class ThermalMotorModel():
	def __init__(self, ambient=21, params = dict(), temp_limit_windings=115, soft_border_C_windings=15, temp_limit_case=80, soft_border_C_case=5):
		self.__dict__.update(default_thermal_parameters)
		self.__dict__.update(params)
		self.T_w = ambient
		self.T_c = ambient
		self.T_a = ambient
		self.soft_max_temp_windings = temp_limit_windings-soft_border_C_windings
		self.abs_max_temp_windings = temp_limit_windings
		self.soft_border_windings = soft_border_C_windings


		self.soft_max_temp_case = temp_limit_case-soft_border_C_case
		self.abs_max_temp_case = temp_limit_case
		self.soft_border_case = soft_border_C_case

	def update_only(self, dt, I_q_des):
		## Dynamics:
		# self.C_w * d self.T_w /dt = I2R + (self.T_c-self.T_w)/self.R_WC
		# self.C_c * d self.T_c /dt = (self.T_w-self.T_c)/self.R_WC + (self.T_w-self.T_a)/self.R_CA

		I2R = I_q_des**2*self.R_ϕ_0*(1+self.α*(self.T_w-self.R_T_0)) # accounts for resistance change due to temp.

		dTw_dt = (I2R + (self.T_c-self.T_w)/self.R_WC)/self.C_w
		dTc_dt = ((self.T_w-self.T_c)/self.R_WC + (self.T_a-self.T_c)/self.R_CA)/self.C_c
		self.T_w += dt*dTw_dt
		self.T_c += dt*dTc_dt


	def update_and_get_scale(self, dt, I_q_des, FOS=3.):
		## Dynamics:
		# self.C_w * d self.T_w /dt = I2R + (self.T_c-self.T_w)/self.R_WC
		# self.C_c * d self.T_c /dt = (self.T_w-self.T_c)/self.R_WC + (self.T_w-self.T_a)/self.R_CA
		I2R_des = FOS*I_q_des**2*self.R_ϕ_0*(1+self.α*(self.T_w-self.R_T_0)) # accounts for resistance change due to temp.
		scale=1.0
		if self.T_w > self.abs_max_temp_windings:
			scale = 0.0
		elif self.T_w > self.soft_max_temp_windings:
			scale *= (self.abs_max_temp_windings - self.T_w)/(self.abs_max_temp_windings - self.soft_max_temp_windings)


		if self.T_c > self.abs_max_temp_case:
			scale = 0.0
		elif self.T_c > self.soft_max_temp_case:
			scale *= (self.abs_max_temp_case - self.T_w)/(self.abs_max_temp_case - self.soft_max_temp_case)

		I2R = I2R_des*scale

		dTw_dt = (I2R + (self.T_c-self.T_w)/self.R_WC)/self.C_w
		dTc_dt = ((self.T_w-self.T_c)/self.R_WC + (self.T_a-self.T_c)/self.R_CA)/self.C_c
		self.T_w += dt*dTw_dt
		self.T_c += dt*dTc_dt

		if scale<=0.0:
			return 0.0
		if scale>=1.0:
			return 1.0
		return np.sqrt(scale) # this is how much the torque should be scaled


def test_temp_limit():
	ENGAGE_SAFETY_LIMIT = False
	tmm = ThermalMotorModel()
	t0=0
	data=[]
	for t in np.linspace(0, 60*60*4, 10000):
		dt=t-t0
		t0=t
		max_continuous_current = 8.336 # T-motor torque, using our model to predict current
		# max_continuous_current = 12*np.sqrt(2/3) # [dubious] T-motor rated current of 12 A (presumably ll), converted to q-axis
		i_des = max_continuous_current if t>60 else 0 
		if ENGAGE_SAFETY_LIMIT:
			scale = tmm.update_and_get_scale(dt, i_des)
			i = scale * i_des
		else:
			tmm.update_only(dt, i_des)
			i = i_des
		data.append([t, i, i_des, tmm.T_w, tmm.T_c])
	data = np.array(data)
	import matplotlib.pyplot as plt
	plt.plot(data[:,0], data[:,2], 'k', label='des current')
	plt.plot(data[:,0], data[:,1], lw=2, label='current')
	plt.plot(data[:,0], data[:,3], label='T_w')
	plt.plot(data[:,0], data[:,4], label='T_c')
	plt.title("Predicted temps @ T-motor max continuous torque")
	plt.ylabel("temp (C)")
	plt.xlabel("time (s)")
	plt.legend()
	plt.show()

def demo_safe_saturation():
	ENGAGE_SAFETY_LIMIT = True
	tmm = ThermalMotorModel()
	t0=0
	data=[]
	for t in np.linspace(0, 20, 20*100):
		dt=t-t0
		t0=t
		pulse_current = 35.52 # current required to achieve 30Nm with antagonistic friction
		# max_continuous_current = 12*np.sqrt(2/3) # [dubious] T-motor rated current of 12 A (presumably ll), converted to q-axis
		i_des = pulse_current if t>10 else 0 
		
		if ENGAGE_SAFETY_LIMIT:
			scale = tmm.update_and_get_scale(dt, i_des)
			i = scale * i_des
		else:
			tmm.update_only(dt, i_des)
			i = i_des
		data.append([t, i, i_des, tmm.T_w, tmm.T_c])
	data = np.array(data)
	import matplotlib.pyplot as plt
	fig,axs = plt.subplots(2,1,sharex=True)
	axs[1].plot(data[:,0], data[:,2], 'k', label='des current')
	axs[1].plot(data[:,0], data[:,1], lw=2, label='current')
	axs[0].plot(data[:,0], data[:,3], label='T_w')
	axs[0].plot(data[:,0], data[:,4], label='T_c')
	axs[0].set_title("Safety saturation demo")
	axs[0].set_ylabel("temp (C)")
	axs[1].set_ylabel("q-axis current (A)")
	axs[-1].set_xlabel("time (s)")
	axs[0].legend()
	axs[1].legend()
	plt.show()

if __name__ == '__main__':
	# test_temp_limit()
	demo_safe_saturation()

