#!/usr/bin/env python3
from flexbe_core import EventState, Logger
import time
import numpy
import cv2
from cv2 import aruco
import pickle
import glob, os
import configparser
from sensor_msgs.msg import CameraInfo
from ament_index_python.packages import get_package_share_directory

# ==================== PARCHE DE COMPATIBILIDAD PARA OPENCV 4.11.0 ====================
def create_charuco_board(col_count, row_count, square_length, marker_length, dictionary):
    """
    Función unificada para crear CharucoBoard compatible con TODAS las versiones de OpenCV
    """
    try:
        # OpenCV 4.11.0 - Usa 'size' como parámetro
        board = aruco.CharucoBoard(
            size=(col_count, row_count),
            squareLength=square_length,
            markerLength=marker_length,
            dictionary=dictionary
        )
        Logger.loginfo("✅ Usando API OpenCV 4.11.0 (con parámetro 'size')")
        return board
    except TypeError as e:
        try:
            # OpenCV 4.8+ - Usa squaresX, squaresY
            board = aruco.CharucoBoard(
                squaresX=col_count,
                squaresY=row_count,
                squareLength=square_length,
                markerLength=marker_length,
                dictionary=dictionary
            )
            Logger.loginfo("✅ Usando API OpenCV 4.8+ (con squaresX, squaresY)")
            return board
        except TypeError:
            # OpenCV < 4.8 - Usa CharucoBoard_create
            Logger.loginfo("✅ Usando API OpenCV antigua (CharucoBoard_create)")
            return aruco.CharucoBoard_create(
                squaresX=col_count,
                squaresY=row_count,
                squareLength=square_length,
                markerLength=marker_length,
                dictionary=dictionary
            )

def get_predefined_dictionary(dict_id):
    """
    Función unificada para obtener diccionario compatible con TODAS las versiones
    """
    try:
        # OpenCV 4.8+
        return aruco.getPredefinedDictionary(dict_id)
    except AttributeError:
        try:
            # OpenCV < 4.8
            return aruco.Dictionary_get(dict_id)
        except AttributeError:
            # Último recurso: usar DICT_4X4_100 (valor 3)
            Logger.logwarn("⚠️ Usando diccionario por defecto DICT_4X4_100")
            return aruco.getPredefinedDictionary(aruco.DICT_4X4_100)
# ====================================================================================

class CharucoCameraCalibrationState(EventState):
    """
    Procesa imágenes de charuco board para calibrar la cámara.
    
    <= done                                    Calibración exitosa
    <= failed                                   Error en calibración
    """
    
    def __init__(self, square_size, marker_size, col_count, row_count, save_file_name):
        """Constructor"""
        super(CharucoCameraCalibrationState, self).__init__(outcomes=['done', 'failed'])
        self.camera_calibration_file = save_file_name
        
        self.CHARUCOBOARD_COLCOUNT = col_count
        self.CHARUCOBOARD_ROWCOUNT = row_count    
        
        # Obtener diccionario de forma compatible
        try:
            self.ARUCO_DICT = get_predefined_dictionary(aruco.DICT_4X4_100)
        except:
            Logger.logwarn("⚠️ Usando DICT_4X4_100 por defecto")
            self.ARUCO_DICT = aruco.DICT_4X4_100
        
        self.squareLength = square_size
        self.markerLength = marker_size

        # Crear CharucoBoard de forma compatible con TODAS las versiones
        self.CHARUCO_BOARD = create_charuco_board(
            col_count=self.CHARUCOBOARD_COLCOUNT,
            row_count=self.CHARUCOBOARD_ROWCOUNT,
            square_length=self.squareLength,
            marker_length=self.markerLength,
            dictionary=self.ARUCO_DICT
        )

        # Create the arrays and variables we'll use to store info like corners and IDs from images processed
        self.corners_all = [] # Corners discovered in all images processed
        self.ids_all = [] # Aruco ids corresponding to corners discovered
        self.image_size = None # Determined at runtime
        
        # Obtener ruta de imágenes
        try:
            self.save_pwd = get_package_share_directory('charuco_detector') + '/config/camera_calibration/'
        except:
            self.save_pwd = os.path.expanduser('~/static/drims2_ws/install/charuco_detector/share/charuco_detector/config/camera_calibration/')
        
        # Buscar imágenes
        self.pic_path = self.save_pwd + 'pic/'
        self.images = glob.glob(self.pic_path + 'camera-pic-of-charucoboard-*.jpg')
        
        Logger.loginfo(f"📂 Buscando imágenes en: {self.pic_path}")
        Logger.loginfo(f"📸 Encontradas {len(self.images)} imágenes")

    def on_start(self):
        pass
    
    def execute(self, userdata):
        if not self.images:
            Logger.logerr(f"❌ No se encontraron imágenes en: {self.pic_path}")
            Logger.loginfo("💡 Asegúrate de haber capturado imágenes primero con el estado 'take_camera_cali_pic'")
            return "failed"
            
        Logger.loginfo(f"🔍 Procesando {len(self.images)} imágenes para calibración...")
        
        for i, iname in enumerate(self.images):
            Logger.loginfo(f"📷 Procesando imagen {i+1}/{len(self.images)}: {os.path.basename(iname)}")
            
            # Open the image
            img = cv2.imread(iname)
            if img is None:
                Logger.logwarn(f"⚠️ No se pudo leer la imagen: {iname}")
                continue
                
            # Grayscale the image
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # Find aruco markers in the query image
            corners, ids, _ = aruco.detectMarkers(
                    image=gray,
                    dictionary=self.ARUCO_DICT)

            # Skip if no markers found
            if ids is None or len(ids) == 0:
                Logger.logwarn(f"⚠️ No se encontraron marcadores en: {os.path.basename(iname)}")
                continue

            # Get charuco corners and ids from detected aruco markers
            try:
                response, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
                        markerCorners=corners,
                        markerIds=ids,
                        image=gray,
                        board=self.CHARUCO_BOARD)
            except Exception as e:
                Logger.logwarn(f"⚠️ Error interpolando corners: {str(e)}")
                continue

            # If a Charuco board was found, let's collect image/corner points
            # Requiring at least 20 squares
            if response is not None and response > 20:
                # Add these corners and ids to our calibration arrays
                self.corners_all.append(charuco_corners)
                self.ids_all.append(charuco_ids)
                Logger.loginfo(f"   ✅ Board detectado con {response} esquinas")

                # If our image size is unknown, set it now
                if not self.image_size:
                    self.image_size = gray.shape[::-1]
            else:
                Logger.logwarn(f"⚠️ No se pudo detectar charuco board en: {os.path.basename(iname)} (response={response})")

        # Destroy any open CV windows
        cv2.destroyAllWindows()
        
        # Make sure we were able to calibrate on at least one charucoboard
        if not self.image_size or len(self.corners_all) < 3:
            Logger.logerr(f"❌ Calibración fallida. Solo se detectaron {len(self.corners_all)} boards válidos (mínimo 3)")
            return "failed"

        # Now that we've seen all of our images, perform the camera calibration
        Logger.loginfo(f"📊 Calibrando cámara con {len(self.corners_all)} boards válidos...")
        
        try:
            calibration, cameraMatrix, distCoeffs, rvecs, tvecs = aruco.calibrateCameraCharuco(
                    charucoCorners=self.corners_all,
                    charucoIds=self.ids_all,
                    board=self.CHARUCO_BOARD,
                    imageSize=self.image_size,
                    cameraMatrix=None,
                    distCoeffs=None)

            print("\n" + "="*50)
            print("✅ CALIBRACIÓN EXITOSA")
            print("="*50)
            print("\n📷 Matriz de Cámara (Intrínsecos):")
            print(cameraMatrix)
            print("\n📏 Coeficientes de Distorsión:")
            print(distCoeffs)
            print("="*50 + "\n")
            
            # Guardar resultados
            config = configparser.ConfigParser()
            config.optionxform = str 
            
            config.add_section("Distortion")
            config.set("Distortion", "k1",  str(distCoeffs[0][0]))
            config.set("Distortion", "k2",  str(distCoeffs[0][1]))
            config.set("Distortion", "t1",  str(distCoeffs[0][2]))
            config.set("Distortion", "t2",  str(distCoeffs[0][3]))
            config.set("Distortion", "k3",  str(distCoeffs[0][4]))
            
            config.add_section("Intrinsic")
            config.set("Intrinsic", "0_0", str(cameraMatrix[0][0]))
            config.set("Intrinsic", "0_1", str(cameraMatrix[0][1]))
            config.set("Intrinsic", "0_2", str(cameraMatrix[0][2]))
            config.set("Intrinsic", "1_0", str(cameraMatrix[1][0]))
            config.set("Intrinsic", "1_1", str(cameraMatrix[1][1]))
            config.set("Intrinsic", "1_2", str(cameraMatrix[1][2]))
            config.set("Intrinsic", "2_0", str(cameraMatrix[2][0]))
            config.set("Intrinsic", "2_1", str(cameraMatrix[2][1]))
            config.set("Intrinsic", "2_2", str(cameraMatrix[2][2]))
            
            # Asegurar que el directorio existe
            os.makedirs(self.save_pwd, exist_ok=True)
            
            output_file = self.save_pwd + self.camera_calibration_file
            with open(output_file, 'w') as file:
                config.write(file)

            Logger.loginfo(f'💾 Archivo guardado: {output_file}')
            
            return 'done'
            
        except Exception as e:
            Logger.logerr(f"❌ Calibración fallida: {str(e)}")
            import traceback
            traceback.print_exc()
            return "failed"
            
    def on_enter(self, userdata):
        pass
