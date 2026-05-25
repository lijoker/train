# AGENTS.md

## Cursor Cloud specific instructions

### Project overview

This is a single-file Python script (`huoche.py`) that automates train ticket purchasing on China's 12306.cn railway website using `splinter` (a browser-automation wrapper around Selenium) and Chrome.

### Dependencies

- Python 3.x with `splinter` and `selenium` packages (installed via `pip install splinter selenium`)
- Google Chrome (pre-installed in the VM)
- ChromeDriver is auto-managed by Selenium 4.x — no manual installation needed

### Running the script

The script (`huoche.py`) hardcodes `executable_path='D:/chrom/chromedriver'` (a Windows path). In the Cloud VM, the Selenium 4.x built-in driver manager handles ChromeDriver automatically, so `executable_path` is not needed when using the modern splinter API. To test browser automation without modifying the repo, create a separate test script that uses `Browser('chrome', options=chrome_options)` with headless Chrome options.

To run headless Chrome via splinter in the Cloud VM:

```python
from splinter import Browser
from selenium.webdriver.chrome.options import Options

chrome_options = Options()
chrome_options.add_argument('--headless')
chrome_options.add_argument('--no-sandbox')
chrome_options.add_argument('--disable-dev-shm-usage')

browser = Browser('chrome', options=chrome_options)
```

### Key caveats

- The script requires a valid 12306.cn account (username/password) and involves manual CAPTCHA solving — full end-to-end runs are not automatable without credentials and human input.
- The script was originally written for a Windows environment. The `executable_path` in `__init__` is a Windows path and will fail on Linux; use Selenium 4.x's built-in driver management instead.
- There are no automated tests, no linter config, and no build system in this repo.
- Xvfb is available in the VM for non-headless browser testing if needed: `Xvfb :99 -screen 0 1400x1000x24 &` then `export DISPLAY=:99`.
