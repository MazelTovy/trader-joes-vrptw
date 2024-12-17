import pandas as pd
import folium
from folium import plugins

def create_map():
    # 读取包含经纬度的 CSV 文件
    df = pd.read_csv("trader_joes_nyc_stores.csv")
    
    # 过滤掉没有经纬度的行
    df = df.dropna(subset=['Latitude', 'Longitude'])
    
    # 计算所有位置的中心点作为地图中心
    center_lat = df['Latitude'].mean()
    center_lon = df['Longitude'].mean()
    
    # 创建地图对象
    nyc_map = folium.Map(
        location=[center_lat, center_lon],
        zoom_start=11,
        tiles='cartodbpositron'  # 使用清晰的地图样式
    )
    
    # 添加每个 Trader Joe's 位置的标记
    for idx, row in df.iterrows():
        # 创建弹出窗口内容
        popup_content = f"""
        <div style="width:200px">
            <b>{row['Store_Name']}</b><br>
            地址: {row['Street_Address']}<br>
            {row['City_Name']}, {row['State']} {row['Zip_Code']}<br>
            电话: {row['Phone']}
        </div>
        """
        
        # 添加标记
        folium.Marker(
            location=[row['Latitude'], row['Longitude']],
            popup=folium.Popup(popup_content, max_width=300),
            icon=folium.Icon(color='red', icon='info-sign'),
            tooltip=row['Store_Name']
        ).add_to(nyc_map)
    
    # 添加位置聚类
    marker_cluster = plugins.MarkerCluster().add_to(nyc_map)
    
    # 添加图例
    legend_html = '''
    <div style="position: fixed; 
                bottom: 50px; right: 50px; 
                border:2px solid grey; z-index:9999;
                background-color:white;
                padding: 10px;
                font-size:14px;">
        <b>Trader Joe's Positions</b><br>
        <i class="fa fa-map-marker fa-2x" style="color:red"></i> Store Positions
    </div>
    '''
    nyc_map.get_root().html.add_child(folium.Element(legend_html))
    
    # 保存地图
    nyc_map.save('trader_joes_nyc_locations.html')
    print("地图已保存为 trader_joes_nyc_locations.html")

if __name__ == "__main__":
    create_map()