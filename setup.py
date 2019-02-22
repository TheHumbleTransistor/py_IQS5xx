from setuptools import setup

setup(name='IQS5xx',
      version='1.0.0',
      description='Python code for programming and controlling Azoteq\'s IQS5xx series of capacitive touch controllers',
      url='https://github.com/TheHumbleTransistor/py_IQS5xx',
      author='Ray Kampmeier',
      author_email='ray@thehumbletransistor.com',
      license='MIT License',
      packages=['IQS5xx'],
      include_package_data=True,
      zip_safe=False,
      install_requires=[
          'usb_iss~=0.3.1',
          'pyserial==3.4'
      ]
      )
