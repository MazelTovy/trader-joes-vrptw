from selenium import webdriver
from selenium.webdriver.chrome.service import Service
from selenium.webdriver.common.by import By
from selenium.webdriver.chrome.options import Options
from selenium.webdriver.support.ui import WebDriverWait
from selenium.webdriver.support import expected_conditions as EC
import pandas as pd
import time

# 设置 ChromeDriver 路径
chromedriver_path = "/Users/alexxu/Documents/NYU/chow/project/scratch/chromedriver-mac-arm64/chromedriver"

# Chrome 设置
options = Options()
options.binary_location = "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"

# 初始化 WebDriver 服务
service = Service(executable_path=chromedriver_path)
driver = webdriver.Chrome(service=service, options=options)

# 纽约州门店页面 URL
base_url = "https://locations.traderjoes.com/ny/"
driver.get(base_url)
time.sleep(3)  # 等待页面加载

# Step 1: 提取所有城市链接
print("开始提取城市列表...")
city_elements = driver.find_elements(By.CSS_SELECTOR, ".itemlist")

city_links = []
for city in city_elements:
    try:
        # 获取城市名称和链接
        city_link = city.find_element(By.CSS_SELECTOR, "a.ga_w2gi_lp")
        # 从 data-linktrack 属性中提取城市名称
        data_linktrack = city_link.get_attribute("data-linktrack")
        city_name = data_linktrack.split(" - ")[-1].strip()
        city_href = city_link.get_attribute("href")
        city_links.append((city_name, city_href))
        print(f"找到城市: {city_name}, 链接: {city_href}")
    except Exception as e:
        print(f"提取城市链接时出错: {e}")

print(f"共找到 {len(city_links)} 个城市链接")

# Step 2: 遍历每个城市页面，提取门店信息
store_data = []

for city_name, city_link in city_links:
    try:
        print(f"正在访问城市页面: {city_name} ({city_link})")
        driver.get(city_link)
        time.sleep(2)  # 等待页面加载

        # 查找所有门店列表项
        store_items = driver.find_elements(By.CLASS_NAME, "address-left")
        
        for store in store_items:
            try:
                # 提取店铺名称
                store_name = store.find_element(By.CSS_SELECTOR, "span.ga_w2gi_lp").text.strip()
                
                # 提取地址组件
                spans = store.find_elements(By.TAG_NAME, "span")
                address_components = []
                for span in spans:
                    if span.get_attribute("class") != "ga_w2gi_lp":  # 排除店铺名称的span
                        text = span.text.strip()
                        if text:  # 只添加非空文本
                            address_components.append(text)
                
                # 提取电话号码
                try:
                    phone = store.find_element(By.CLASS_NAME, "phone-btn").text.strip()
                except:
                    phone = "N/A"

                # 保存数据
                store_data.append({
                    "City": city_name,
                    "Store_Name": store_name,
                    "Street_Address": address_components[0] if len(address_components) > 0 else "N/A",
                    "City_Name": address_components[1] if len(address_components) > 1 else "N/A",
                    "State": address_components[2] if len(address_components) > 2 else "N/A",
                    "Zip_Code": address_components[3] if len(address_components) > 3 else "N/A",
                    "Country": address_components[4] if len(address_components) > 4 else "N/A",
                    "Phone": phone
                })
                
                print(f"提取门店信息: {store_name}")

            except Exception as e:
                print(f"提取门店详细信息时出错: {e}")
                continue

    except Exception as e:
        print(f"访问城市页面 {city_name} 时出错: {e}")
        continue

# 保存数据到 CSV 文件
df = pd.DataFrame(store_data)
df.to_csv("trader_joes_ny_stores.csv", index=False, encoding="utf-8")
print("数据已保存到 trader_joes_ny_stores.csv")

# 关闭浏览器
driver.quit()