import sys
import rospy

# Ensure the path is added
sys.path.append('/home/ruan-x/midTask/devel/lib/python3/dist-packages')

def main():
    rospy.init_node('test_import')
    
    try:
        from openai_test.srv import SelectTopic, SelectTopicResponse
        rospy.loginfo("Import successful!")
    except ImportError as e:
        rospy.logerr(f"Import failed: {e}")
    
    rospy.spin()

if __name__ == '__main__':
    main()
