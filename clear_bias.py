import time
import sys
from selenium import webdriver
import os
from colorama import init
import telegram
from datetime import datetime
from selenium.common.exceptions import NoSuchElementException
from selenium.webdriver.common.by import By
from selenium.webdriver.support.ui import Select
from decimal import Decimal, ROUND_DOWN
from multiprocessing import Process


class Setting:
    def __init__(self):
        self.DRIVER_PATH = '/home/shichen/Guder/PriceChecker/SeleniumDrivers/Linux/chromedriver'

        self.user_agent = 'Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/60.0.3112.50 Safari/537.36'
        self.options = webdriver.ChromeOptions()
        self.options.add_experimental_option('excludeSwitches', ['enable-logging'])
        # self.options.add_argument("--headless")
        self.options.add_argument("--window-size=1920,1080")
        self.options.add_argument(f"user-agent={self.user_agent}")
        self.options.add_argument("--disable-popup-blocking")
        self.options.add_argument("start-maximized")
        self.options.add_argument("--headless")
        self.options.add_argument('--ignore-certificate-errors')
        self.options.add_argument('--allow-running-insecure-content')
        self.options.add_argument("--incognito")
        self.options.add_argument("--no-sandbox")
        self.options.add_argument("--disable-gpu")
        self.no_stock = ['no', 'not', 'sold', 'out']
        self.in_stock = ['cart', 'add']
        self.api_key = '2113396884:AAHb8L8mYY2EgFB3XV0pqSqa2GhReQxQuUo'
        self.laodao_id = '-1001179455742'
        self.eth_id = '-1001413275954'
        self.my_id = '1961467450'


class ATI(webdriver.Chrome):
    def __init__(self, st: Setting, url, teardown=False):
        self.st = st
        super(ATI, self).__init__(executable_path=self.st.DRIVER_PATH, options=self.st.options)
        # self.delete_all_cookies()
        self.url = url
        self.implicitly_wait(2)

    def load_page(self):
        self.get(self.url)

    def reset_bias(self):
        bias = self.find_element(By.XPATH,
                                 '/html/body/table/tbody/tr[2]/td/table/tbody/tr/td[2]/table/tbody/tr[3]/td/form/table/tbody/tr[10]/td/input')
        bias.click()


def main(url):
    bot = ATI(st=Setting(), url=url)
    bot.load_page()
    bot.reset_bias()
    # while True:
    #     pass
    # str_price, price, tier = bot.info()
    # msg = coin + ' : ' + str_price
    # bot.telegram_communicator(msg)
    # # bot.telegram_communicator(coin + ' Price Checker Loaded!')
    # _, _, pre_tier = bot.info()
    # while True:
    #     try:
    #         pre_tier, str_price = bot.monitoring(pre_tier)
    #     except (Exception,):
    #         time.sleep(60)
    #         bot.refresh()
    #         pre_tier, str_price = bot.monitoring(pre_tier)
    #     bot.refresh()
    #     time.sleep(300)


if __name__ == '__main__':
    main('http://airlab:airlab@192.168.125.185/rundata.htm')
