from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import pandas as pd
import numpy as np
import folium
import requests
import polyline

distance_limit = 500

def create_distance_matrix(routes_df, stores_df):
    """创建距离矩阵并保存到文件"""
    distance_matrix_file = 'distance_matrix.npy'
    try:
        # 尝试从文件加载距离矩阵
        distances = np.load(distance_matrix_file)
        print("已从文件加载距离矩阵")
        return distances
    except:
        print("创建新的距离矩阵...")
        n_stores = len(stores_df)
        distances = np.zeros((n_stores, n_stores))
        
        # 初始化为一个较大的值
        distances.fill(float('inf'))
        np.fill_diagonal(distances, 0)  # 对角线设为0
        
        # 填充距离矩阵
        for _, row in routes_df.iterrows():
            store1_idx = stores_df[stores_df['Store_Name'] == row['store1']].index[0]
            store2_idx = stores_df[stores_df['Store_Name'] == row['store2']].index[0]
            distances[store1_idx][store2_idx] = row['distance_km']
            distances[store2_idx][store1_idx] = row['distance_km']  # 确保矩阵对称
        
        # 保存距离矩阵到文件
        np.save(distance_matrix_file, distances)
        return distances

def identify_distant_stores(stores_df):
    """识别距离配送中心太远的店铺并保存结果"""
    distant_stores_file = 'distant_stores.npy'
    try:
        # 尝试从文件加载远距离店铺列表
        distant_stores = np.load(distant_stores_file)
        print("已从文件加载远距离店铺列表")
        return list(distant_stores)
    except:
        print("识别远距离店铺...")
        distant_stores = []
        depot_coords = stores_df.iloc[0]
        
        for idx, store in stores_df.iloc[1:].iterrows():
            route_data = get_route(
                (depot_coords['Longitude'], depot_coords['Latitude']),
                (store['Longitude'], store['Latitude'])
            )
            if route_data:
                round_trip_distance = 2 * route_data['distance'] / 1000
                if round_trip_distance > distance_limit:
                    distant_stores.append(idx)
                    print(f"店铺 {store['Store_Name']} 往返距离为 {round_trip_distance:.2f} km")
        
        # 保存结果到文件
        np.save(distant_stores_file, np.array(distant_stores))
        return distant_stores

def solve_vrp(distance_matrix, demands, vehicle_capacity, num_vehicles, excluded_indices=None):
    """使用 OR-Tools 解决 VRP 问题"""
    if excluded_indices is None:
        excluded_indices = []
    
    manager = pywrapcp.RoutingIndexManager(len(distance_matrix), num_vehicles, 0)
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        if to_node in excluded_indices:
            return 999999999
        return int(distance_matrix[from_node][to_node] * 1000)

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    def demand_callback(from_index):
        from_node = manager.IndexToNode(from_index)
        if from_node in excluded_indices:
            return 0
        return demands[from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,
        [vehicle_capacity] * num_vehicles,
        True,
        'Capacity'
    )

    # 使用更宽松的距离约束
    dimension_name = 'Distance'
    routing.AddDimension(
        transit_callback_index,
        0,
        distance_limit * 1000,  # 使用完整的距离限制
        True,
        dimension_name
    )
    distance_dimension = routing.GetDimensionOrDie(dimension_name)

    for vehicle_id in range(num_vehicles):
        routing.SetFixedCostOfVehicle(100000, vehicle_id)

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    )
    search_parameters.time_limit.FromSeconds(30)  # 减少求解时间

    solution = routing.SolveWithParameters(search_parameters)
    return manager, routing, solution

def get_route(start_coords, end_coords):
    """获取两点间的路径"""
    lon1, lat1 = start_coords
    lon2, lat2 = end_coords
    
    url = f"http://router.project-osrm.org/route/v1/driving/{lon1},{lat1};{lon2},{lat2}?overview=full&geometries=polyline"
    
    try:
        response = requests.get(url)
        if response.status_code == 200:
            route_data = response.json()
            if route_data["code"] == "Ok":
                distance = route_data["routes"][0]["distance"]
                duration = route_data["routes"][0]["duration"]
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

def get_routes(manager, routing, solution, stores_df, distance_matrix):
    """提取路由结果"""
    routes = []
    
    for vehicle_id in range(routing.vehicles()):
        index = routing.Start(vehicle_id)
        route = []
        route_distance = 0
        previous_node = None
        
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            route.append(stores_df.iloc[node_index]['Store_Name'])
            
            if previous_node is not None:
                route_distance += distance_matrix[previous_node][node_index]
            
            previous_node = node_index
            index = solution.Value(routing.NextVar(index))
        
        # 添加返回起点的距离
        if previous_node is not None:
            route_distance += distance_matrix[previous_node][0]
        
        # 检查总距离是否超过限制
        if len(route) > 2 and route_distance > distance_limit:  # 不是单店配送
            return None
        
        route.append(stores_df.iloc[0]['Store_Name'])
        routes.append({
            'vehicle_id': vehicle_id,
            'route': route,
            'total_distance': route_distance
        })
    
    return routes

def visualize_routes(routes, stores_df):
    """在地图上可视化路由结果"""
    center_lat = stores_df['Latitude'].mean()
    center_lon = stores_df['Longitude'].mean()
    nyc_map = folium.Map(
        location=[center_lat, center_lon],
        zoom_start=11,
        tiles='cartodbpositron'
    )
    
    # 创建路径图层组
    route_groups = {}
    distance_ranges = [
        (0, 5), (5, 10), (10, 15), (15, 20), (20, float('inf'))
    ]
    
    for start, end in distance_ranges:
        group_name = f"{start}-{end if end != float('inf') else '+'} km"
        route_groups[group_name] = folium.FeatureGroup(name=group_name)
        nyc_map.add_child(route_groups[group_name])
    
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd']
    
    for i, route_info in enumerate(routes):
        color = colors[i % len(colors)]
        route = route_info['route']
        
        for j in range(len(route) - 1):
            start_store = route[j]
            end_store = route[j + 1]
            
            start_coords = stores_df[stores_df['Store_Name'] == start_store].iloc[0]
            end_coords = stores_df[stores_df['Store_Name'] == end_store].iloc[0]
            
            route_data = get_route(
                (start_coords['Longitude'], start_coords['Latitude']),
                (end_coords['Longitude'], end_coords['Latitude'])
            )
            
            if route_data:
                distance_km = route_data['distance'] / 1000
                
                for (start_range, end_range) in distance_ranges:
                    if start_range <= distance_km < end_range:
                        group_name = f"{start_range}-{end_range if end_range != float('inf') else '+'} km"
                        break
                
                folium.PolyLine(
                    locations=route_data['coords'],
                    weight=3,
                    color=color,
                    opacity=0.8,
                    popup=f"车辆 {i + 1}: {start_store} → {end_store}<br>"
                          f"距离: {distance_km:.2f} km<br>"
                          f"时间: {route_data['duration']/60:.0f} 分钟<br>"
                          f"{'返回起点' if j == len(route)-2 else ''}",
                    tooltip=f"车辆 {i + 1}: {distance_km:.1f} km"
                ).add_to(route_groups[group_name])
    
    # 添加门店标记
    for _, store in stores_df.iterrows():
        is_depot = (store['Store_Name'] == stores_df.iloc[0]['Store_Name'])
        
        folium.Marker(
            location=[store['Latitude'], store['Longitude']],
            popup=folium.Popup(
                f"""<div style="width:200px">
                    <b>{store['Store_Name']}</b><br>
                    {'<b>配送中心</b><br>' if is_depot else ''}
                    地址: {store['Street_Address']}<br>
                    {store['City_Name']}, {store['State']} {store['Zip_Code']}<br>
                    电话: {store['Phone']}
                </div>""",
                max_width=300
            ),
            icon=folium.Icon(
                color='green' if is_depot else 'red',
                icon='star' if is_depot else 'info-sign'
            ),
            tooltip=f"{'配送中心: ' if is_depot else ''}{store['Store_Name']}"
        ).add_to(nyc_map)
    
    folium.LayerControl().add_to(nyc_map)
    
    return nyc_map

def main():
    # 读取数据
    routes_df = pd.read_csv('trader_joes_routes.csv')
    stores_df = pd.read_csv('trader_joes_ny_stores_with_coords.csv')
    
    # 添加布鲁克林第五码头作为起点
    depot = pd.DataFrame({
        'Store_Name': ['Brooklyn Pier 5'],
        'Street_Address': ['Brooklyn Bridge Park Pier 5'],
        'City_Name': ['Brooklyn'],
        'State': ['NY'],
        'Zip_Code': ['11201'],
        'Phone': ['N/A'],
        'Latitude': [40.7003],
        'Longitude': [-73.9690]
    })
    
    # 将配送中心添加到门店数据的开头
    stores_df = pd.concat([depot, stores_df]).reset_index(drop=True)
    
    # 识别远距离店铺
    distant_stores = identify_distant_stores(stores_df)
    print(f"\n发现 {len(distant_stores)} 个远距离店铺需要专门配送")
    for idx in distant_stores:
        print(f"- {stores_df.iloc[idx]['Store_Name']}")
    
    # 创建距离矩阵
    distance_matrix = create_distance_matrix(routes_df, stores_df)
    
    # 设置参数
    WEEKLY_DEMAND = 500  # 每店每周需求
    VEHICLE_CAPACITY = 3000  # 每车容量
    demands = [0] + [WEEKLY_DEMAND] * (len(stores_df) - 1)  # 配送中心需求为0
    
    # 计算剩余店铺所需车辆数
    remaining_stores = len(stores_df) - 1 - len(distant_stores)  # 减去配送中心和远距离店铺
    total_demand = WEEKLY_DEMAND * remaining_stores
    num_vehicles = max(3, -(-total_demand // VEHICLE_CAPACITY))
    solution = None
    routes = None
    
    # 解决主要的VRP问题（不包括远距离店铺）
    while solution is None and num_vehicles <= remaining_stores:
        print(f"\n尝试使用 {num_vehicles} 辆车解决主要配送问题...")
        manager, routing, solution = solve_vrp(
            distance_matrix, 
            demands, 
            VEHICLE_CAPACITY,
            num_vehicles,
            distant_stores
        )
        
        if solution:
            routes = get_routes(manager, routing, solution, stores_df, distance_matrix)
            if routes is None:  # 如果实际距离超过限制
                solution = None
                num_vehicles += 1
                continue
        else:
            num_vehicles += 1
    
    if solution and routes:
        # 为远距离店铺添加专门的路线
        for idx in distant_stores:
            store = stores_df.iloc[idx]
            route_data = get_route(
                (stores_df.iloc[0]['Longitude'], stores_df.iloc[0]['Latitude']),
                (store['Longitude'], store['Latitude'])
            )
            if route_data:
                round_trip_distance = 2 * route_data['distance'] / 1000
                routes.append({
                    'vehicle_id': len(routes),
                    'route': [stores_df.iloc[0]['Store_Name'], store['Store_Name'], stores_df.iloc[0]['Store_Name']],
                    'total_distance': round_trip_distance
                })
        
        # 打印结果
        print(f"\n=== 配送方案 ===")
        print(f"配送中心: Brooklyn Pier 5")
        print(f"使用车辆总数: {len(routes)}")
        total_distance = 0
        
        # 先打印常规路线
        for route in routes[:-len(distant_stores)]:
            print(f"\n常规车辆 {route['vehicle_id'] + 1}:")
            print(f"路线: {' -> '.join(route['route'])}")
            print(f"总距离: {route['total_distance']:.2f} km")
            total_distance += route['total_distance']
        
        # 再打印远离路线
        for route in routes[-len(distant_stores):]:
            print(f"\n专门配送车辆 {route['vehicle_id'] + 1}:")
            print(f"路线: {' -> '.join(route['route'])}")
            print(f"总距离: {route['total_distance']:.2f} km")
            print("(此路线超出常规距离限制)")
            total_distance += route['total_distance']
            
        print(f"\n总行驶距离: {total_distance:.2f} km")
        
        # 可视化路线
        m = visualize_routes(routes, stores_df)
        m.save('vrp_brooklyn_solution.html')
        print("\n路线图已保存至 vrp_brooklyn_solution.html")
    else:
        print("未找到满足所有约束的可行解")

if __name__ == "__main__":
    main()
    main()