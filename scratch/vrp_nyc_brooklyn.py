from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import pandas as pd
import numpy as np
import folium
from folium import plugins
import itertools
import requests
import polyline

def create_distance_matrix(routes_df, stores_df):
    """创建距离矩阵"""
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
    
    return distances

def create_data_model(distance_matrix, demands, num_vehicles):
    """创建数据模型"""
    data = {}
    
    # 转换距离为时间（假设平均速度30km/h）
    time_matrix = []
    for row in distance_matrix:
        time_row = []
        for d in row:
            if np.isinf(d):
                time_row.append(99999999)
            else:
                time_row.append(int(d * 60 / 30))  # 转换为分钟
        time_matrix.append(time_row)
    
    data['time_matrix'] = time_matrix
    data['distance_matrix'] = distance_matrix
    data['demands'] = demands
    data['num_vehicles'] = num_vehicles
    data['depot'] = 0
    
    # 设置时间窗口（分钟，相对于0点）
    DEPOT_WINDOW = (0, 1440)  # 配送中心24小时
    STORE_WINDOW = (0, 1440)  # 店铺允许24小时送达
    
    data['time_windows'] = [DEPOT_WINDOW]  # 配送中心
    data['time_windows'].extend([STORE_WINDOW] * (len(distance_matrix) - 1))  # 其他店铺
    
    # 调整时间段及其成本权重
    data['time_penalties'] = {
        'night': {'start': 0, 'end': 480, 'weight': -30},      # 0:00-8:00 适度奖励
        'day': {'start': 480, 'end': 1260, 'weight': 0},       # 8:00-21:00 无惩罚
        'evening': {'start': 1260, 'end': 1440, 'weight': -20} # 21:00-24:00 小奖励
    }
    
    # 等待惩罚（每分钟）
    data['waiting_penalty'] = 2
    
    return data

def get_waiting_time_penalty(arrival_time, data):
    """计算等待时间惩罚"""
    arrival_time = arrival_time % 1440  # 转换为24小时制
    
    # 如果到达时间在白天（8:00-21:00），无需等待
    if 480 <= arrival_time < 1260:
        return 0
        
    # 如果在夜间到达，计算需要等待到晚上21:00的时间
    if arrival_time < 480:
        waiting_time = 1260 - arrival_time
    else:  # arrival_time >= 1260
        waiting_time = 0
    
    return waiting_time * data['waiting_penalty']

def solve_vrp(distance_matrix, demands, vehicle_capacity, num_vehicles):
    """使用 OR-Tools 解决 VRP 问题"""
    data = create_data_model(distance_matrix, demands, num_vehicles)
    
    manager = pywrapcp.RoutingIndexManager(len(distance_matrix), num_vehicles, 0)
    routing = pywrapcp.RoutingModel(manager)

    # 定义距离成本回调
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return int(data['distance_matrix'][from_node][to_node] * 100)  # 转换为整数并加权

    distance_callback_index = routing.RegisterTransitCallback(distance_callback)

    # 定义时间回调
    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['time_matrix'][from_node][to_node]

    time_callback_index = routing.RegisterTransitCallback(time_callback)

    # 定义考虑时间奖惩的成本回调
    def total_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        
        # 基础距离成本
        distance_cost = data['distance_matrix'][from_node][to_node] * 100
        
        # 如果是返回配送中心，只计算距离成本
        if to_node == 0:
            return int(distance_cost)
            
        # 获取预计到达时间
        current_time = routing.CumulVar(from_index, "Time").Max()
        arrival_time = current_time + data['time_matrix'][from_node][to_node]
        arrival_time = arrival_time % 1440
        
        # 计算时间奖惩
        time_cost = 0
        for period, info in data['time_penalties'].items():
            if info['start'] <= arrival_time < info['end']:
                time_cost = info['weight']
                break
        
        # 计算等待惩罚
        waiting_penalty = get_waiting_time_penalty(arrival_time, data)
        
        return int(distance_cost + time_cost + waiting_penalty)

    total_callback_index = routing.RegisterTransitCallback(total_callback)
    
    # 使用总成本作为主要评估标准
    routing.SetArcCostEvaluatorOfAllVehicles(total_callback_index)

    # 添加容量约束
    def demand_callback(from_index):
        from_node = manager.IndexToNode(from_index)
        return demands[from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,
        [vehicle_capacity] * num_vehicles,
        True,
        'Capacity'
    )

    # 添加时间维度
    routing.AddDimension(
        time_callback_index,
        30,  # 允许等待时间减少到30分钟
        1440,  # 最大时间（24小时）
        False,  # 不强制从0开始
        'Time'
    )
    
    time_dimension = routing.GetDimensionOrDie('Time')
    
    # 为每个位置添加时间窗口约束
    for location_idx, time_window in enumerate(data['time_windows']):
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # 设置求解策略
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    )
    search_parameters.time_limit.FromSeconds(30)

    solution = routing.SolveWithParameters(search_parameters)
    return manager, routing, solution, data

def get_time_period(minutes):
    """获取时间段描述"""
    if 0 <= minutes < 480:  # 0:00-8:00
        return "夜间配送"
    elif 480 <= minutes < 1260:  # 8:00-21:00
        return "营业时间"
    else:  # 21:00-24:00
        return "晚间配送"

def format_time(minutes):
    """将分钟数转换为时间格式"""
    # 处理超过24小时的情况
    minutes = minutes % 1440
    hours = minutes // 60
    mins = minutes % 60
    return f"{hours:02d}:{mins:02d}"

def get_time_cost(time_minutes, data):
    """计算时间奖惩成本"""
    time_minutes = time_minutes % 1440  # 转换为24小时制
    for period, info in data['time_penalties'].items():
        if info['start'] <= time_minutes < info['end']:
            return info['weight']
    return 0

def get_routes(manager, routing, solution, stores_df, data):
    """提取路由结果"""
    routes = []
    time_dimension = routing.GetDimensionOrDie('Time')
    
    total_distance = 0
    total_time_cost = 0
    
    for vehicle_id in range(routing.vehicles()):
        index = routing.Start(vehicle_id)
        route = []
        route_distance = 0
        route_time_cost = 0
        previous_node = None
        
        while not routing.IsEnd(index):
            time_var = time_dimension.CumulVar(index)
            node_index = manager.IndexToNode(index)
            
            # 获取到达时间
            min_time = solution.Min(time_var)
            max_time = solution.Max(time_var)
            
            # 获取时间段描述和成本
            period = get_time_period(min_time)
            time_cost = get_time_cost(min_time, data)
            route_time_cost += time_cost
            
            route.append({
                'store': stores_df.iloc[node_index]['Store_Name'],
                'arrival_time': (format_time(min_time), format_time(max_time)),
                'period': period,
                'time_cost': time_cost,
                'address': stores_df.iloc[node_index]['Street_Address'],
                'phone': stores_df.iloc[node_index]['Phone']
            })
            
            if previous_node is not None:
                route_distance += data['distance_matrix'][previous_node][node_index]
            
            previous_node = node_index
            index = solution.Value(routing.NextVar(index))
        
        # 添加返回配送中心的信息
        time_var = time_dimension.CumulVar(index)
        min_time = solution.Min(time_var)
        max_time = solution.Max(time_var)
        period = get_time_period(min_time)
        
        if previous_node is not None:
            route_distance += data['distance_matrix'][previous_node][0]
        
        route.append({
            'store': stores_df.iloc[0]['Store_Name'],
            'arrival_time': (format_time(min_time), format_time(max_time)),
            'period': period,
            'time_cost': 0,  # 返回配送中心不计时间成本
            'address': stores_df.iloc[0]['Street_Address'],
            'phone': stores_df.iloc[0]['Phone']
        })
        
        total_distance += route_distance
        total_time_cost += route_time_cost
        
        routes.append({
            'vehicle_id': vehicle_id,
            'route': route,
            'total_distance': route_distance,
            'time_cost': route_time_cost
        })
    
    return routes, total_distance, total_time_cost

def visualize_routes(routes, stores_df):
    """在地图上可视化路由结果"""
    # 创建地图
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
    
    # 为每个车辆使用不同颜色
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd']
    
    # 添加每个路线
    for i, route_info in enumerate(routes):
        color = colors[i % len(colors)]
        route = route_info['route']
        
        # 绘制路线
        for j in range(len(route) - 1):
            start_store = route[j]['store']
            end_store = route[j + 1]['store']
            
            # 获取起点和终点的坐标
            start_store_data = stores_df[stores_df['Store_Name'] == start_store]
            end_store_data = stores_df[stores_df['Store_Name'] == end_store]
            
            if len(start_store_data) == 0 or len(end_store_data) == 0:
                print(f"警告: 找不到店铺 {start_store} 或 {end_store}")
                continue
            
            start_coords = start_store_data.iloc[0]
            end_coords = end_store_data.iloc[0]
            
            # 获取路径
            route_data = get_route(
                (start_coords['Longitude'], start_coords['Latitude']),
                (end_coords['Longitude'], end_coords['Latitude'])
            )
            
            if route_data:
                distance_km = route_data['distance'] / 1000
                
                # 确定路径所属距离组
                for (start_range, end_range) in distance_ranges:
                    if start_range <= distance_km < end_range:
                        group_name = f"{start_range}-{end_range if end_range != float('inf') else '+'} km"
                        break
                
                # 添加路径线
                folium.PolyLine(
                    locations=route_data['coords'],
                    weight=3,
                    color=color,
                    opacity=0.8,
                    popup=f"车辆 {i + 1}: {start_store} → {end_store}<br>"
                          f"距离: {distance_km:.2f} km<br>"
                          f"时间: {route_data['duration']/60:.0f} 分钟<br>"
                          f"到达时间: {route[j+1]['arrival_time'][0]}<br>"
                          f"时间段: {route[j+1]['period']}",
                    tooltip=f"车辆 {i + 1}: {distance_km:.1f} km"
                ).add_to(route_groups[group_name])
    
    # 添加门店标记
    for _, store in stores_df.iterrows():
        # 判断是否为配送中心（第一个店）
        is_depot = (store['Store_Name'] == stores_df.iloc[0]['Store_Name'])
        
        folium.Marker(
            location=[store['Latitude'], store['Longitude']],
            popup=folium.Popup(
                f"""<div style="width:200px">
                    <b>{store['Store_Name']}</b><br>
                    {'<b>Delivery Center</b><br>' if is_depot else ''}
                    Address: {store['Street_Address']}<br>
                    {store['City_Name']}, {store['State']} {store['Zip_Code']}<br>
                    Phone number: {store['Phone']}
                </div>""",
                max_width=300
            ),
            icon=folium.Icon(
                color='green' if is_depot else 'red',
                icon='star' if is_depot else 'info-sign'
            ),
            tooltip=f"{'Delivery Center: ' if is_depot else ''}{store['Store_Name']}"
        ).add_to(nyc_map)
    
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
        <b>Trader Joe's delivery routes</b><br>
        <i class="fa fa-star fa-2x" style="color:green"></i> Delivery Center<br>
        <i class="fa fa-map-marker fa-2x" style="color:red"></i> Store<br>
        <div style="margin-top: 5px;">Vehicles:</div>
    ''' 
    
    for i in range(len(routes)):
        legend_html += f'<div style="color:{colors[i % len(colors)]}">● Vehicle {i + 1}</div>'
    
    legend_html += '</div>'
    nyc_map.get_root().html.add_child(folium.Element(legend_html))
    
    return nyc_map

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

def main():
    # 读取数据
    routes_df = pd.read_csv('trader_joes_nyc_routes.csv')
    stores_df = pd.read_csv('trader_joes_nyc_stores.csv')
    
    # 创建距离矩阵
    distance_matrix = create_distance_matrix(routes_df, stores_df)
    
    # 设置参数
    WEEKLY_DEMAND = 500  # 每店每周需求
    VEHICLE_CAPACITY = 3000  # 每车容量
    demands = [0] + [WEEKLY_DEMAND] * (len(stores_df) - 1)  # 配送中心需求为0
    
    # 计算所需车辆数
    total_demand = sum(demands)
    min_vehicles = max(3, -(-total_demand // VEHICLE_CAPACITY))
    num_vehicles = min_vehicles * 2  # 使用适度数量的车辆
    
    print(f"最小车辆数（基于容量）: {min_vehicles}")
    print(f"实际使用车辆数: {num_vehicles}")
    
    # 固定车辆数量为4-5辆
    num_vehicles = 4  # 或者5，可以尝试两种情况
    
    # 多次运行找到最优解
    best_solution = None
    best_manager = None
    best_routing = None
    best_data = None
    best_total_cost = float('inf')
    
    num_attempts = 5
    
    for attempt in range(num_attempts):
        print(f"\n尝试 {attempt + 1}/{num_attempts}")
        
        manager, routing, solution, data = solve_vrp(
            distance_matrix, 
            demands, 
            VEHICLE_CAPACITY,
            num_vehicles
        )
        
        if solution:
            routes, total_distance, total_time_cost = get_routes(manager, routing, solution, stores_df, data)
            total_cost = total_distance * 100 + total_time_cost  # 距离成本权重为100
            
            print(f"当前解总成本: {total_cost:.2f}")
            
            if total_cost < best_total_cost:
                best_total_cost = total_cost
                best_solution = solution
                best_manager = manager
                best_routing = routing
                best_data = data
                print("找到新的最优解!")
    
    if best_solution:
        routes, total_distance, total_time_cost = get_routes(
            best_manager, best_routing, best_solution, stores_df, best_data
        )
        
        print(f"\n=== 最优配送方案 ===")
        print(f"配送中心: Brooklyn Pier 5")
        
        # 统计不同时间段的配送数量和成本
        period_stats = {
            '夜间配送': {'count': 0, 'cost': 0},
            '营业时间': {'count': 0, 'cost': 0},
            '晚间配送': {'count': 0, 'cost': 0}
        }
        
        for route in routes:
            print(f"\n车辆 {route['vehicle_id'] + 1}:")
            print("配送路线:")
            for stop in route['route']:
                period_stats[stop['period']]['count'] += 1
                period_stats[stop['period']]['cost'] += stop['time_cost']
                print(f"  {stop['store']}")
                print(f"    地址: {stop['address']}")
                print(f"    电话: {stop['phone']}")
                print(f"    到达时间: {stop['arrival_time'][0]}-{stop['arrival_time'][1]}")
                print(f"    时间段: {stop['period']}")
                print(f"    时间成本: {stop['time_cost']}")
            print(f"路线距离: {route['total_distance']:.2f} km")
            print(f"路线时间成本: {route['time_cost']}")
        
        print(f"\n=== 统计信息 ===")
        print(f"总行驶距离: {total_distance:.2f} km")
        print(f"总时间成本: {total_time_cost}")
        print(f"总成本: {best_total_cost:.2f}")
        
        print("\n配送时间段分布:")
        for period, stats in period_stats.items():
            print(f"{period}: {stats['count']} 次配送, 总成本: {stats['cost']}")
        
        # 可视化最优路线
        m = visualize_routes(routes, stores_df)
        m.save('vrp_brooklyn_tw_solution.html')
        print("\n路线图已保存至 vrp_brooklyn_tw_solution.html")
    else:
        print("未找到可行解，请尝试调整参数")

if __name__ == "__main__":
    main()