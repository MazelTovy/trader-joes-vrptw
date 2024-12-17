import pandas as pd
import folium
from folium import plugins
import requests
import itertools
from tqdm import tqdm
import polyline
import numpy as np

def get_route(start_coords, end_coords):
    """获取两点间的路径"""
    lon1, lat1 = start_coords
    lon2, lat2 = end_coords
    
    # 使用 OSRM 服务
    url = f"http://router.project-osrm.org/route/v1/driving/{lon1},{lat1};{lon2},{lat2}?overview=full&geometries=polyline"
    
    try:
        response = requests.get(url)
        if response.status_code == 200:
            route_data = response.json()
            if route_data["code"] == "Ok":
                # 获取距离（米）和时间（秒）
                distance = route_data["routes"][0]["distance"]
                duration = route_data["routes"][0]["duration"]
                # 获取路径坐标
                geometry = route_data["routes"][0]["geometry"]
                route_coords = polyline.decode(geometry)
                return {
                    "distance": distance,
                    "duration": duration,
                    "coords": route_coords
                }
    except Exception as e:
        print(f"获取路径时出错: {e}")
    return None

def create_route_map():
    # 读取门店数据
    df = pd.read_csv("trader_joes_nyc_stores.csv")
    df = df.dropna(subset=['Latitude', 'Longitude'])
    
    # 创建地图
    center_lat = df['Latitude'].mean()
    center_lon = df['Longitude'].mean()
    nyc_map = folium.Map(
        location=[center_lat, center_lon],
        zoom_start=11,
        tiles='cartodbpositron'
    )
    
    # 存储所有路径信息
    routes_info = []
    
    # 获取所有可能的门店配对
    store_pairs = list(itertools.combinations(df.index, 2))
    
    # 使用进度条显示计算进度
    for pair in tqdm(store_pairs, desc="计算路径"):
        store1 = df.iloc[pair[0]]
        store2 = df.iloc[pair[1]]
        
        # 获取路径
        route = get_route(
            (store1['Longitude'], store1['Latitude']),
            (store2['Longitude'], store2['Latitude'])
        )
        
        if route:
            routes_info.append({
                'store1': store1['Store_Name'],
                'store2': store2['Store_Name'],
                'distance': route['distance'],
                'duration': route['duration'],
                'coords': route['coords']
            })
    
    # 添加门店标记
    for idx, row in df.iterrows():
        popup_content = f"""
        <div style="width:200px">
            <b>{row['Store_Name']}</b><br>
            地址: {row['Street_Address']}<br>
            {row['City_Name']}, {row['State']} {row['Zip_Code']}<br>
            电话: {row['Phone']}
        </div>
        """
        
        folium.Marker(
            location=[row['Latitude'], row['Longitude']],
            popup=folium.Popup(popup_content, max_width=300),
            icon=folium.Icon(color='red', icon='info-sign'),
            tooltip=row['Store_Name']
        ).add_to(nyc_map)
    
    # 创建路径图层组
    route_groups = {}
    distance_ranges = [
        (0, 5), (5, 10), (10, 15), (15, 20), (20, float('inf'))
    ]
    
    for start, end in distance_ranges:
        group_name = f"{start}-{end if end != float('inf') else '+'} km"
        route_groups[group_name] = folium.FeatureGroup(name=group_name)
        nyc_map.add_child(route_groups[group_name])
    
    # 绘制所有路径，按距离分组
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', 
              '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf']
    
    for route in routes_info:
        distance_km = route['distance'] / 1000
        
        # 确定路径所属的距离组
        for (start, end) in distance_ranges:
            if start <= distance_km < end:
                group_name = f"{start}-{end if end != float('inf') else '+'} km"
                break
        
        # 根据距离设置线条颜色和透明度
        opacity = max(0.2, min(0.8, 1 - (distance_km / 30)))  # 距离越远透明度越高
        weight = max(1, min(3, 3 - (distance_km / 20)))  # 距离越远线条越细
        
        # 添加路径线
        folium.PolyLine(
            locations=route['coords'],
            weight=weight,
            color=colors[distance_ranges.index((start, end)) % len(colors)],
            opacity=opacity,
            popup=f"距离: {distance_km:.2f}km<br>"
                  f"时间: {route['duration']/60:.0f}分钟<br>"
                  f"从 {route['store1']}<br>到 {route['store2']}",
            tooltip=f"{distance_km:.1f} km"
        ).add_to(route_groups[group_name])
    
    # 添加图层控制
    folium.LayerControl().add_to(nyc_map)
    
    # 添加图例
    legend_html = '''
    <div style="position: fixed; 
                bottom: 50px; right: 50px; 
                border:2px solid grey; z-index:9999;
                background-color:white;
                padding: 10px;
                font-size:14px;">
        <b>Trader Joe's routes</b><br>
        <i class="fa fa-map-marker fa-2x" style="color:red"></i> Store Positions<br>
        <div style="margin-top: 5px;">Distance:</div>
        <div style="color:#1f77b4">● 0-5 km</div>
        <div style="color:#ff7f0e">● 5-10 km</div>
        <div style="color:#2ca02c">● 10-15 km</div>
        <div style="color:#d62728">● 15-20 km</div>
        <div style="color:#9467bd">● 20+ km</div>
    </div>
    '''
    nyc_map.get_root().html.add_child(folium.Element(legend_html))
    
    # 保存地图
    nyc_map.save('trader_joes_nyc_all_routes.html')
    
    # 保存路径信息到 CSV
    routes_df = pd.DataFrame(routes_info)
    routes_df['distance_km'] = routes_df['distance'] / 1000
    routes_df['duration_min'] = routes_df['duration'] / 60
    routes_df[['store1', 'store2', 'distance_km', 'duration_min']].to_csv(
        'trader_joes_nyc_routes.csv', index=False
    )

if __name__ == "__main__":
    create_route_map()