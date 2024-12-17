import pandas as pd
from geopy.geocoders import Nominatim
from geopy.exc import GeocoderTimedOut, GeocoderServiceError
import time

def get_coordinates(address):
    """
    使用 Nominatim 获取地址的经纬度
    """
    geolocator = Nominatim(user_agent="trader_joes_locator")
    try:
        # 添加延时避免请求过快
        time.sleep(1)
        location = geolocator.geocode(address)
        if location:
            return location.latitude, location.longitude
        return None, None
    except (GeocoderTimedOut, GeocoderServiceError) as e:
        print(f"地理编码错误: {e}")
        return None, None

def main():
    # 读取现有的 CSV 文件
    try:
        df = pd.read_csv("trader_joes_ny_stores.csv")
        print("成功读取 CSV 文件")
    except Exception as e:
        print(f"读取 CSV 文件时出错: {e}")
        return

    # 添加经纬度列
    df['Latitude'] = None
    df['Longitude'] = None

    # 遍历每行数据获取经纬度
    for index, row in df.iterrows():
        # 构建完整地址
        address = f"{row['Street_Address']}, {row['City_Name']}, {row['State']} {row['Zip_Code']}, USA"
        print(f"正在获取地址的经纬度: {address}")
        
        # 获取经纬度
        lat, lon = get_coordinates(address)
        
        # 更新数据框
        df.at[index, 'Latitude'] = lat
        df.at[index, 'Longitude'] = lon
        
        print(f"经度: {lon}, 纬度: {lat}")

    # 保存更新后的 CSV 文件
    try:
        df.to_csv("trader_joes_ny_stores_with_coords.csv", index=False, encoding="utf-8")
        print("数据已保存到 trader_joes_ny_stores_with_coords.csv")
    except Exception as e:
        print(f"保存 CSV 文件时出错: {e}")

if __name__ == "__main__":
    main()