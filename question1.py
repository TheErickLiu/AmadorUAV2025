import pandas as pd
from sklearn.cluster import KMeans
from sklearn.neighbors import LocalOutlierFactor

def getClusterCenters(filename):
    
    df = pd.read_csv(filename, sep=' ', skiprows=1, header=None, names=["latitude", "longitude"])

    lof = LocalOutlierFactor()
    lof.fit_predict(df[["latitude", "longitude"]])

    df["outlier"] = (lof.negative_outlier_factor_ < -2.0).astype(int)

    clean_df = df[df["outlier"] == 0]
    k_means = KMeans(n_clusters=5, random_state=0, n_init="auto").fit(clean_df)

    return k_means.cluster_centers_

def run():
    centers = getClusterCenters("with outliers.txt")
    print(centers)
    with open('centers.out', 'w') as f:
        for center in centers:
            f.write(f"{center[0]} {center[1]}\n")

run()