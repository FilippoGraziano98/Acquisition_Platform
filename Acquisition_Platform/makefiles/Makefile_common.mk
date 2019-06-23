
SENSORS += -DUSE_IMU
SENSORS += -DUSE_ENCS

ODOM += -DODOM_ENCS
ODOM += -DODOM_IMU

#KF_VERSION=0
#		use delta_l and delta_r as measurements from encoders
#		using Taylor polinomial in the observation_matrix

#KF_VERSION=1
#		preprocess encoder data
#		using local_dx, local_dy and local_dtheta as measurements

ODOM += -DKF_VERSION=1
ODOM += -DKF_IMU				#to include also IMU, in sensor observation passed to KF

#DEBUG_OPTS += -DDEBUG_PRINTF		#use cutecom, no packet handling
#DEBUG_OPTS += -DDEBUG_ODOM
DEBUG_OPTS += -DDEBUG_IMU_CALIB
#DEBUG_OPTS += -DDEBUG_KF_MATRIXES
#DEBUG_OPTS += -DDEBUG_KF_OBS


COMMON_VARS += -DSYNCHRONIZATION_CYCLES=10
