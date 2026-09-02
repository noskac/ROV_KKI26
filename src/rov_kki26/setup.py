from setuptools import find_packages, setup

package_name = 'rov_kki26'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cakson',
    maintainer_email='andaruwicak04@gmail.com',
    description='Sistem Ground Control Station ROV MAIVS EVO — KKI 2026',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'video_receiver = rov_kki26.video_receiver_node:main',
            'rov_dashboard = rov_kki26.rov_dashboard_node:main',
            'telemetry_receiver = rov_kki26.telemetry_receiver_node:main',
            'qr_scanner = rov_kki26.qr_scanner_node:main',
            'mavis_gamepad = rov_kki26.mavis_gamepad_node:main',
            'data_logger = rov_kki26.data_logger_node:main',
            # 'video_recorder' DIHAPUS — perekaman video sekarang ada di dalam
            # video_receiver_node.py. Node perekam terpisah menambah proses
            # subscriber ketiga ke topic image_raw (~75 MB/detik per proses),
            # yang membuat frame besar gagal terkirim lewat DDS dan video di
            # GUI hilang total. Penjelasan lengkap ada di docstring
            # video_receiver_node.py.
        ],
    },
)