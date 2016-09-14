from flickrapi import FlickrAPI
import urllib
import urllib2
import requests
import threading
from pprint import pprint

FLICKR_PUBLIC = 'ba4923af4bc8e9aa01186259dd238090'
FLICKR_SECRET = '23a28f84942610b8'

textqry="india monuments"
per_page_qry = 4
lim_pages = 10

flickr = FlickrAPI(FLICKR_PUBLIC, FLICKR_SECRET, format='parsed-json')
extras='url_m'
cats = flickr.photos.search(text=textqry, per_page=per_page_qry, extras=extras)
photos = cats['photos']

def downloadpage(page_no):
  flickr = FlickrAPI(FLICKR_PUBLIC, FLICKR_SECRET, format='parsed-json')
  cats = flickr.photos.search(text=textqry, per_page=per_page_qry, extras=extras,page=page_no)
  photos = cats['photos']
  image_id=0
  for photo in photos['photo']:
    print "photo id downloading", image_id
    image_id+=1
    url = photo['url_m']
    f = urllib2.urlopen(url)
    data = f.read()
    with open("code" +str(page_no)+"-"+str(image_id) + ".jpg", "wb") as code:
        code.write(data)


def download_pages(arr):
  for elem in arr:
    downloadpage(elem)

no_iterations = photos['pages']
no_iterations=min(lim_pages, no_iterations)
pagids = [i for i in xrange(1,no_iterations+1)]
print pagids
numthreads=4
chunksize = no_iterations/numthreads

threads=[]
for i in xrange(numthreads):
  print "Starting thread", i
  arrrequired = pagids[chunksize*i:min(chunksize*(i+1),no_iterations)]
  # thread.start_new_thread(download_pages, (arrrequired,))
  # thread.join()
  t = threading.Thread(target=download_pages, args=(arrrequired,))
  threads.append(t)

for t in threads:
  t.start()

for t in threads:
  t.join()

# threads = []
# for i in range(numthreads):
#   print "Starting thread", i
#   t = threading.Thread(target=worker, args=(i,))
#     threads.append(t)
#     t.start()

# x= raw_input()
# for i in xrange(1,no_iterations+1):
#   print "Downloading iteration: ", i, "of", str(no_iterations+1) 
#   cats = flickr.photos.search(text=textqry, per_page=per_page_qry, extras=extras,page=i)
#   photos = cats['photos']
#   image_id=0
#   for photo in photos['photo']:
#     print "photo id downloading", image_id
#     image_id+=1
#     url = photo['url_m']
#     f = urllib2.urlopen(url)
#     data = f.read()
#     with open("code" +str(i)+"-"+str(image_id) + ".jpg", "wb") as code:
#         code.write(data)
