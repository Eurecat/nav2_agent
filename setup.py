from glob import glob
from setuptools import find_packages, setup

package_name = 'nav2_agent'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/config', glob('config/*.yaml')),
        (f'share/{package_name}/launch', glob('launch/*.launch.py')),
    ],
    install_requires=[
        'setuptools',
        'pydantic',
        'pydantic-ai',
        'openai',
        'PyYAML',
    ],
    zip_safe=True,
    maintainer='Nav2 Agent Maintainer',
    maintainer_email='user@example.com',
    description='PydanticAI-based ROS 2 navigation command agent for a local vLLM backend.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav2_agent_node = nav2_agent.agent_node:main',
        ],
    },
)
