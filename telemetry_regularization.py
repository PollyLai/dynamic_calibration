import pandas as pd

df = pd.read_csv("ur10_simulation_telemetry.csv")
# toScale = ["q", "qd", "i", "id", "tau"]
toScale = ["i", "id", "tau"]
scale = [1, 1, 1, 1, 1, 870]
for currScale in toScale:
    for index, scaleNum in enumerate(scale):
        scaling = currScale + str(index + 1)
        print(f'scaling {scaling} by {scaleNum}')
        df[scaling] /= (scaleNum)
print(df)
df.to_csv("ur10_simulation_telemetry_regularized.csv", index=False)