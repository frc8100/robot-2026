import pandas as pd
import statsmodels.api as sm

# Load the CSV data from the log file (make sure to change the path as needed)
df = pd.read_csv("C:\\Users\\Public\\wpilib\\2025\\logs\\akit_25-11-16_15-49-02.csv")

X = pd.DataFrame({
    "sgn": df["/RealOutputs/SysId/FFCharacterization/VelocitySign"],
    "velocity": df["/RealOutputs/SysId/FFCharacterization/VelocityRadPerSec"],
    # "accel": df["/RealOutputs/SysId/FFCharacterization/ChassisAccelMPS2"],
    "force": df["/RealOutputs/SysId/FFCharacterization/LinearForcesFFVolts"]
})

# X = sm.add_constant(X, has_constant="add")  # optional constant term

model = sm.OLS(df["/RealOutputs/SysId/FFCharacterization/AppliedVoltage"], X).fit()
print(model.summary())
