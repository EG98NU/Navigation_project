# Navigation project
Here you can find the code I wrote to develope the project explained carefully in the report.

## Execution steps

1) Run 'traj_gen.m' to generate the trajectory and the true values of acceleration, velocity and angular velocity in navigation frame and position.

2) Run 'init_sim.m' to generate all the parameters necessary for the simulation execution.

3) Open and run 'SGN_sim.slx' which contains the sensors models based on the true values extracted from 'traj_gen.m', and the extended Kalman filters that estimates the navigation variables using the sensor outputs. Inside the simulation there are some scopes that allow to compare the variables.

4) Run 'SGN_plots.m' to see the plots of the difference between the variables of navigation calculated throught just mechanization and the same variables calculated through the Kalman filters.

['Jacobians_att.m' shows how 'F_attitude.m', 'D_attitude.m' and 'H_attitude.m' were generated and computes the rank of the osservability matrix for the attitude. 'Jacobians_pv.m' does not generate any file but computes the rank of the osservability matrix for position and velocity.]





