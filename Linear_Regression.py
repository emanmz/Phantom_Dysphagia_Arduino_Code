import pandas as pd
from scipy.stats import linregress
import io

# Data provided by the user, formatted for easy parsing
data_string = """
Cmd,Measured
2.8,1.746787
2.9,1.839224
3.0,1.925911
3.1,2.012495
3.2,2.106331
3.3,2.193329
3.4,2.280379
3.5,2.375406
3.6,2.463181
3.7,2.555152
3.8,2.646786
3.9,2.736504
4.0,2.829744
4.1,2.916224
4.2,3.005812
"""

# Read the data into a pandas DataFrame
df = pd.read_csv(io.StringIO(data_string))

# Define X (Commanded) and Y (Measured) variables
X = df['Cmd']
Y = df['Measured']

# Perform linear regression: Y = mX + b
slope, intercept, r_value, p_value, std_err = linregress(X, Y)

# Calculate R-squared value
r_squared = r_value**2

# Print the results
print(f"Slope (m): {slope:.6f}")
print(f"Intercept (b): {intercept:.6f}")
print(f"R-squared (R^2): {r_squared:.6f}")