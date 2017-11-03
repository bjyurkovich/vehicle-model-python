import numpy as np
import scipy.io as spio
import matplotlib
import matplotlib.pyplot as plt
import sys

from driver import Driver
from controller import Controller
from vehicle import Vehicle
from battery_pack import Battery
from electric_motor import ElectricMotor
from gearbox import Gearbox
from front_brakes import FrontBrakes
from rear_brakes import RearBrakes
from front_wheels import FrontWheels
from rear_wheels import RearWheels

delta_t = 1
vehicle_mass = 3000 * 0.453592  # in kg

k_p = 0.1
k_i = 0.03  # 0.6
k_d = 0  # 2

c_drag = 0.24
frontal_area = 3.5
em_efficiency = 0.95
em_gearbox_ratio = 18.63

brake_rear_proportion = 0.4
brake_front_proportion = 0.6

tire_radius = 0.465

driver = Driver(k_p, k_i, k_d, delta_t)
vehicle = Vehicle(vehicle_mass, c_drag, frontal_area, delta_t)
battery = Battery(0.6, 3, 120, delta_t)
em = ElectricMotor()
em_gearbox = Gearbox(em_efficiency, em_gearbox_ratio)
rear_brakes = RearBrakes(brake_rear_proportion, tire_radius)
front_brakes = FrontBrakes(brake_front_proportion, tire_radius)
rear_wheels = RearWheels(tire_radius)
front_wheels = FrontWheels(tire_radius)


mat = spio.loadmat('data/v_cyc.mat', squeeze_me=True)

v_cyc = np.array(mat['v_cyc'])
t_cyc = np.array(mat['t_cyc'])
# print(t_cyc)


v_out = []
power_req_out = []
battery_power_out = []
em_torque_out = []
front_wheel_torque_out = []
rear_wheel_torque_out = []
force_at_wheel_out = []
front_brake_torque_out = []
rear_brake_torque_out = []
alpha = []
beta = []
v_p = []
v_i = []

for v in v_cyc:
    driver_out = driver.compute_step(v, vehicle.velocity)

    # compute necessary battery power
    power_req = driver.alpha * battery.compute_max_power()
    power_req_out.append(power_req)

    b_out = battery.compute_step(power_req)
    battery_power_out.append(b_out['pack_power'])

    em.compute_step(battery.p_pack, em_gearbox.em_force_w)
    em_torque_out.append(em.em_torque)

    rear_brakes.compute_step(rear_wheels.wheel_torque, driver.beta)
    front_brakes.compute_step(front_wheels.wheel_torque, driver.beta)

    em_gearbox.compute_step(em.em_torque, front_brakes.front_brake_w)

    front_wheels.compute_step(
        em_gearbox.torque_out / 2, front_brakes.brake_torque, vehicle.velocity)

    rear_wheels.compute_step(
        em_gearbox.torque_out / 2, rear_brakes.brake_torque, vehicle.velocity)

    vehicle_data = vehicle.compute_step(front_wheels.force_at_wheel +
                                        rear_wheels.force_at_wheel, 0)

    # print(v, vehicle_data['velocity'])
    front_wheel_torque_out.append(front_wheels.wheel_torque)
    rear_wheel_torque_out.append(rear_wheels.wheel_torque)
    force_at_wheel_out.append(front_wheels.force_at_wheel +
                              rear_wheels.force_at_wheel)
    front_brake_torque_out.append(front_brakes.brake_torque)
    rear_brake_torque_out.append(rear_brakes.brake_torque)
    alpha.append(driver.alpha)
    beta.append(driver.beta)
    v_p.append(driver_out['v_p'])
    v_i.append(driver_out['v_i'])
    v_out.append(vehicle_data['velocity'])


fig2 = plt.figure()
plt.title('battery power request')
plt.plot(t_cyc, battery_power_out, 'b')
plt.plot(t_cyc, power_req_out, 'r--')
# plt.legend('battery', 'power')


fig3 = plt.figure()
plt.plot(t_cyc, em_torque_out, 'b')


fig4 = plt.figure()
plt.title('wheel torque')
plt.plot(t_cyc, rear_wheel_torque_out, 'b')
plt.plot(t_cyc, front_wheel_torque_out, 'r--')


fig5 = plt.figure()
plt.title('wheel force')
plt.plot(t_cyc, force_at_wheel_out, 'b')


fig6 = plt.figure()
plt.title('brake torque')
plt.plot(t_cyc, front_brake_torque_out, 'b')
plt.plot(t_cyc, rear_brake_torque_out, 'r--')

fig6 = plt.figure()
plt.title('throttle/brake command')
plt.plot(t_cyc, alpha, 'b')
plt.plot(t_cyc, beta, 'r--')


fig1 = plt.figure()
plt.plot(t_cyc, v_cyc, 'b')
plt.plot(t_cyc, v_out, 'r--')

fig7 = plt.figure(figsize=(8, 9))

veh_speed = plt.subplot(511)
veh_speed.set_title('Vehicle speed')
veh_speed.plot(t_cyc, v_cyc, 'b')
veh_speed.plot(t_cyc, v_out, 'r--')
veh_speed.plot(t_cyc, v_cyc - v_out, 'g')

acc_brake_cmd = plt.subplot(512, sharex=veh_speed)
acc_brake_cmd.set_title('Throttle / Brake command')
acc_brake_cmd.plot(t_cyc, alpha, 'b')
acc_brake_cmd.plot(t_cyc, beta, 'r--')

power_req = plt.subplot(513, sharex=veh_speed)
power_req.set_title('Battery power out')
power_req.plot(t_cyc, em_torque_out)


wheel_torque = plt.subplot(514, sharex=veh_speed)
wheel_torque.set_title('Wheel Torque')
wheel_torque.plot(t_cyc, front_wheel_torque_out, 'b')
wheel_torque.plot(t_cyc, rear_wheel_torque_out, 'r--')

driver_pi = plt.subplot(515, sharex=veh_speed)
driver_pi.set_title('Driver PI')
driver_pi.plot(t_cyc, v_p, 'b')
driver_pi.plot(t_cyc, v_i, 'r')

plt.show()
