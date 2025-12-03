"""
Version management for KaudaV2 Control Application
"""

__version__ = "1.0.0"
__author__ = "Axel Habeillon"
__description__ = "Control application for KaudaV2 5-axis robotic arm"
__license__ = "MIT"
__repository__ = "https://github.com/Axel82/KaudaV2_control"


class Version:
    """Version information for the application"""
    
    MAJOR = 1
    MINOR = 0
    PATCH = 0
    
    @classmethod
    def get_version(cls):
        """Get version string"""
        return f"{cls.MAJOR}.{cls.MINOR}.{cls.PATCH}"
    
    @classmethod
    def get_full_info(cls):
        """Get full version information"""
        return {
            "version": cls.get_version(),
            "author": __author__,
            "description": __description__,
            "license": __license__,
            "repository": __repository__
        }
